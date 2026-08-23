#include <Simulator.hpp>

Simulator::Simulator(Vec2f dims, float maxParticleRadius, float g, float C_r,
                     float dt, size_t maxParticles,
                     IntegrationType integrationType,
                     BroadphaseType broadphaseType, size_t iterations)
    : gravity{g},
      restitution{C_r},
      solverIterations{iterations},
      worldSize_{dims},
      maxParticleRadius_{maxParticleRadius},
      dt_{dt},
      integrationType_{integrationType},
      broadphaseType_{broadphaseType},
      capacity_{maxParticles},
      frameCount_{0},
      currBiggestRadius_{0.0f},
      sumRadii_{0.0f},
      prevAvgRadius_{0.0f} {
  std::random_device rd;
  gen_.seed(rd());
  particles_.reserve(maxParticles);
  contacts_.reserve(maxParticles * 8);
  spatialGrid_.configure(2.0f * maxParticleRadius_, worldSize_);
};

void Simulator::configure(Vec2f size, float dt) {
  worldSize_ = size;
  dt_ = dt;
  spatialGrid_.configure(2.0f * prevAvgRadius_, worldSize_);
}

void Simulator::spawnParticle(Vec2f pos, Vec2f vel, float r, float m) noexcept {
  if (particles_.size() >= capacity_) return;
  const float vn = vel.x * vel.x + vel.y + vel.y;
  if (!vn) {
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    vel = {dist(gen_), dist(gen_)};
  }
  particles_.emplace_back(pos, vel, dt_, r, m);
  currBiggestRadius_ = std::max(currBiggestRadius_, r);
  sumRadii_ += r;
};

void Simulator::radialPush(const Vec2f& origin, const float radius,
                           const float mag) {
  spatialGrid_.queryDoSomething(-1, radius, origin, [&](int neiIdx) {
    Particle& p = particles_[neiIdx];
    const Vec2f d = p.position - origin;
    const float d2 = d.x * d.x + d.y * d.y;

    if (d2 > radius * radius || d2 < 1e-12f) return;

    const float invDist = 1.0f / std::sqrt(d2);
    const Vec2f norm = d * invDist;

    p.accelerate({
        norm.x * mag * (1.0f - std::sqrt(d2) / radius),
        norm.y * mag * (1.0f - std::sqrt(d2) / radius),
    });
  });
}

void Simulator::update() noexcept {
  if (integrationType_ == IntegrationType::Euler) {
    for (Particle& par : particles_) {
      par.accelerate({0.0f, gravity});
      par.integrateEuler(dt_);
    }
  } else {
    for (Particle& par : particles_) {
      par.accelerate({0.0f, gravity});
      par.integrateVerlet(dt_);
    }
  }
  resolveCollisions();
  frameCount_++;
}

void Simulator::naiveBroadphase(std::vector<Contact>& contacts) {
  for (size_t i = 0; i < particles_.size(); i++) {
    for (size_t j = i + 1; j < particles_.size(); j++) {
      detectCollision(i, j, contacts);
    }
  }
}

void Simulator::qtreeBroadphase(std::vector<Contact>& contacts,
                                size_t bucketSize) {
  QuadTree<Particle> qtree(AABBf({0.0f, 0.0f}, {worldSize_.x, worldSize_.y}),
                           bucketSize);
  for (Particle& p : particles_) {
    qtree.insert(&p);
  }

  for (size_t i = 0; i < particles_.size(); i++) {
    Particle& p1 = particles_[i];
    const Vec2f c1 = p1.position;
    const float r1 = p1.radius;
    const float query_r = r1 + currBiggestRadius_;
    const AABBf queryRange({c1.x - query_r, c1.y - query_r},
                           {2.0f * query_r, 2.0f * query_r});

    std::vector<Particle*> neighbors;
    qtree.query(neighbors, queryRange);

    for (Particle* nei : neighbors) {
      size_t j = nei->id;
      if (i >= j) {
        continue;
      }
      detectCollision(i, j, contacts);
    }
  }
}

void Simulator::spatialGridBroadphase(std::vector<Contact>& contacts) {
  float avg_radius = (particles_.size() > 0) ? sumRadii_ / particles_.size()
                                             : maxParticleRadius_;
  if (std::abs(avg_radius - prevAvgRadius_) > 0.2f) {
    float cell_size = 2.0f * avg_radius;
    spatialGrid_.configure(cell_size, worldSize_);
    prevAvgRadius_ = avg_radius;
  }
  spatialGrid_.resize(particles_.size());
  spatialGrid_.build(particles_);

  for (size_t i = 0; i < particles_.size(); i++) {
    Particle& p1 = particles_[i];
    spatialGrid_.queryDoSomething(i, p1.radius, p1.position, [&](int neiIdx) {
      detectCollision(i, static_cast<size_t>(neiIdx), contacts);
    });
  }
}

void Simulator::applyWall(Particle& p, float w, float h) {
  const float r = p.radius;

  if (integrationType_ == IntegrationType::Euler) {
    // top/bot
    if (p.position.y < r) {
      p.position.y = r;
      p.velocity.y = -p.velocity.y * restitution;
    } else if (p.position.y > h - r) {
      p.position.y = h - r;
      p.velocity.y = -p.velocity.y * restitution;
    }

    // left/right
    if (p.position.x < r) {
      p.position.x = r;
      p.velocity.x = -p.velocity.x * restitution;
    } else if (p.position.x > w - r) {
      p.position.x = w - r;
      p.velocity.x = -p.velocity.x * restitution;
    }
  } else {
    float vx = p.position.x - p.prevPosition.x;
    float vy = p.position.y - p.prevPosition.y;

    // top/bot
    if (p.position.y < r) {
      p.position.y = r;
      p.prevPosition.y = p.position.y + vy * restitution;
    } else if (p.position.y > h - r) {
      p.position.y = h - r;
      p.prevPosition.y = p.position.y + vy * restitution;
    }

    // left/right
    if (p.position.x < r) {
      p.position.x = r;
      p.prevPosition.x = p.position.x + vx * restitution;
    } else if (p.position.x > w - r) {
      p.position.x = w - r;
      p.prevPosition.x = p.position.x + vx * restitution;
    }
  }
}

void Simulator::detectCollision(size_t idx1, size_t idx2,
                                std::vector<Contact>& contacts) {
  Particle& p1 = particles_[idx1];
  Particle& p2 = particles_[idx2];

  const Vec2f d = p2.position - p1.position;
  const float d2 = d.x * d.x + d.y * d.y;
  const float sum_r = p1.radius + p2.radius;
  const float sum_r2 = sum_r * sum_r;

  if (d2 >= sum_r2) return;

  if (d2 < 1e-12f) {
    contacts.emplace_back(idx1, idx2, Vec2f{1.0f, 0.0f}, sum_r);
    return;
  }

  const float dist = std::sqrt(d2);
  const Vec2f norm = d / dist;
  const float penetration = sum_r - dist;

  if (penetration > 0.0f) {
    contacts.emplace_back(idx1, idx2, norm, penetration);
  }
}

void Simulator::solveContactsPositionBased(std::vector<Contact>& contacts) {
  for (const Contact& contact : contacts) {
    Particle& p1 = particles_[contact.indexA];
    Particle& p2 = particles_[contact.indexB];

    const float invMassSum = p1.invMass + p2.invMass;
    if (invMassSum < 1e-12f) continue;

    const Vec2f d = p2.position - p1.position;
    const float d2 = d.x * d.x + d.y * d.y;
    const float sum_r = p1.radius + p2.radius;

    if (d2 < 1e-12f) {
      const float half = sum_r * 0.5f;
      p1.position -= contact.normal * (half * (p1.invMass / invMassSum));
      p2.position += contact.normal * (half * (p2.invMass / invMassSum));
      continue;
    }

    const float dist = std::sqrt(d2);
    const Vec2f norm = d / dist;
    const float penetration = sum_r - dist;

    if (penetration > 0.0f) {
      const Vec2f correction = norm * (penetration / invMassSum);
      p1.position -= correction * p1.invMass;
      p2.position += correction * p2.invMass;

      Vec2f v1 = p1.position - p1.prevPosition;
      Vec2f v2 = p2.position - p2.prevPosition;
      const Vec2f relV = v2 - v1;
      const float relVelN = relV.x * norm.x + relV.y * norm.y;

      if (relVelN < 0.0f) {
        const float w1 = p1.invMass / invMassSum;
        const float w2 = p2.invMass / invMassSum;

        const float nRelVelN = -restitution * relVelN;
        const float dRelVelN = nRelVelN - relVelN;

        const Vec2f dV = norm * dRelVelN;
        v1 -= dV * w1;
        v2 += dV * w2;

        p1.prevPosition = p1.position - v1;
        p2.prevPosition = p2.position - v2;
      }
    }
  }
}

void Simulator::solveContactsImpulseBased(std::vector<Contact>& contacts) {
  for (const Contact& contact : contacts) {
    Particle& p1 = particles_[contact.indexA];
    Particle& p2 = particles_[contact.indexB];

    const float invMassSum = p1.invMass + p2.invMass;
    if (invMassSum < 1e-12f) continue;

    const Vec2f d = p2.position - p1.position;
    const float d2 = d.x * d.x + d.y * d.y;
    const float sum_r = p1.radius + p2.radius;

    if (d2 < 1e-12f) {
      const float half = sum_r * 0.5f;
      p1.position -= contact.normal * (half * (p1.invMass / invMassSum));
      p2.position += contact.normal * (half * (p2.invMass / invMassSum));
      continue;
    }

    const float dist = std::sqrt(d2);
    const Vec2f norm = d / dist;
    const float penetration = sum_r - dist;

    if (penetration > 0.0f) {
      const Vec2f correction = norm * (penetration / invMassSum);
      p1.position -= correction * p1.invMass;
      p2.position += correction * p2.invMass;
    }

    const Vec2f relV = p2.velocity - p1.velocity;
    const float relVelN = relV.x * norm.x + relV.y * norm.y;

    if (relVelN < 0.0f) {
      const float j = -(1.0f + restitution) * relVelN / invMassSum;
      const Vec2f impulse = norm * j;
      p1.velocity -= impulse * p1.invMass;
      p2.velocity += impulse * p2.invMass;
    }
  }
}

void Simulator::resolveCollisions() {
  auto [w, h] = worldSize_;

  for (Particle& par : particles_) {
    applyWall(par, w, h);
  }

  for (size_t iter = 0; iter < solverIterations; iter++) {
    contacts_.clear();

    if (broadphaseType_ == BroadphaseType::SpatialGrid) {
      spatialGridBroadphase(contacts_);
    } else if (broadphaseType_ == BroadphaseType::Qtree) {
      qtreeBroadphase(contacts_, 16);
    } else {
      naiveBroadphase(contacts_);
    }

    if (contacts_.empty()) break;

    if (integrationType_ == IntegrationType::Verlet) {
      solveContactsPositionBased(contacts_);
    } else {
      solveContactsImpulseBased(contacts_);
    }
  }
}
