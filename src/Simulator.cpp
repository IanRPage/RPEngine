#include <Simulator.hpp>
#include <cmath>

Simulator::Simulator(Vec2f dims, float maxParticleRadius, float g, float C_r,
                     float dt, IntegrationType integrationType,
                     BroadphaseType broadphaseType, size_t maxParticles)
    : gravity(g),
      restitution(C_r),
      worldSize_(dims),
      maxParticleRadius_(maxParticleRadius),
      dt_(dt),
      integrationType_(integrationType),
      broadphaseType_(broadphaseType),
      capacity_(maxParticles) {
  std::random_device rd;
  gen_.seed(rd());
  particles_.reserve(maxParticles);
  spatialGrid_.configure(2.0f * maxParticleRadius_, worldSize_);
};

void Simulator::configure(Vec2f size, float dt) {
  worldSize_ = size;
  dt_ = dt;
  spatialGrid_.configure(2.0f * maxParticleRadius_, worldSize_);
}

void Simulator::spawnParticle(Vec2f pos, Vec2f vel, float r, float m) noexcept {
  if (particles_.size() >= capacity_) return;
  const float vn = vel.x * vel.x + vel.y + vel.y;
  if (!vn) {
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    vel = {dist(gen_), dist(gen_)};
  }
  particles_.emplace_back(pos, vel, dt_, r, m);
};

void Simulator::radialPush(const Vec2f& origin, const float radius,
                           const float mag, const int scale) {
  spatialGrid_.queryDoSomething(
      -1, origin,
      [&](int neiIdx) {
        Particle& p = particles_[neiIdx];
        const Vec2f d = p.position - origin;
        const float d2 = d.x * d.x + d.y * d.y;

        if (d2 > radius * radius) return;

        const float invDist = 1.0f / std::sqrt(d2);
        const Vec2f norm = d * invDist;

        p.accelerate({norm.x * mag, norm.y * mag});
      },
      scale);
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
}

// O(n^2)
void Simulator::naiveBroadphase() {
  for (size_t i = 0; i < particles_.size(); i++) {
    for (size_t j = i + 1; j < particles_.size(); j++) {
      particleCollision(particles_[i], particles_[j]);
    }
  }
}

// O(nlog(n))
void Simulator::qtreeBroadphase(size_t bucketSize) {
  QuadTree<Particle> qtree(AABBf({0.0f, 0.0f}, {worldSize_.x, worldSize_.y}),
                           bucketSize);
  for (Particle& p : particles_) {
    qtree.insert(&p);
  }

  for (size_t i = 0; i < particles_.size(); i++) {
    Particle& p1 = particles_[i];
    const Vec2f c1 = p1.position;
    const float r1 = p1.radius;
    const AABBf queryRange({c1.x - 2.0f * r1, c1.y - 2.0f * r1},
                           {4.0f * r1, 4.0f * r1});

    std::vector<Particle*> neighbors;
    qtree.query(neighbors, queryRange);

    for (Particle* nei : neighbors) {
      Particle& p2 = *nei;
      if (&p1 <= &p2) {
        continue;
      }
      particleCollision(p1, p2);
    }
  }
}

// O(n)
void Simulator::spatialGridBroadphase() {
  spatialGrid_.resize(particles_.size());
  spatialGrid_.build(particles_);

  // broad-phase
  for (size_t i = 0; i < particles_.size(); i++) {
    Particle& p1 = particles_[i];
    spatialGrid_.queryDoSomething(i, p1.position, [&](int neiIdx) {
      Particle& p2 = particles_[neiIdx];
      particleCollision(p1, p2);
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

void Simulator::particleCollision(Particle& p1, Particle& p2) {
  const Vec2f d = p2.position - p1.position;
  const float d2 = d.x * d.x + d.y * d.y;
  const float sum_r = p1.radius + p2.radius;
  const float sum_r2 = sum_r * sum_r;

  // square dist prune
  if (d2 >= sum_r2) return;

  // if small dist apart
  if (d2 < 1e-12f) {
    Vec2f n = {1.0f, 0.0f};
    const float half = (p1.radius + p2.radius) * 0.5f;

    const float invMassSum = p1.invMass + p2.invMass;
    if (invMassSum > 0.0f) {
      p1.prevPosition = p1.position;
      p2.prevPosition = p2.position;
      p1.position -= n * (half * (p1.invMass / invMassSum));
      p2.position += n * (half * (p2.invMass / invMassSum));
    }
    return;
  }

  const float invDist = 1.0f / std::sqrt(d2);
  const float dist = 1.0f / invDist;
  const Vec2f norm = d * invDist;
  const float penetration = sum_r - dist;

  const float invMassSum = p1.invMass + p2.invMass;

  if (penetration > 0.0f && invMassSum > 0.0f) {
    float percent = 0.30f;
    const Vec2f correction = norm * (percent * penetration / invMassSum);
    p1.position -= correction * p1.invMass;
    p2.position += correction * p2.invMass;
  }

  if (integrationType_ == IntegrationType::Euler) {
    const Vec2f relV = (p2.velocity - p1.velocity);
    const float relVel = relV.x * norm.x + relV.y * norm.y;
    if (relVel < 0) {
      const float magJ = (1.0f + restitution) * relVel / invMassSum;
      const Vec2f J = norm * magJ;
      p1.velocity += J * p1.invMass;
      p2.velocity -= J * p2.invMass;
    }
  } else {
    Vec2f v1 = p1.position - p1.prevPosition;
    Vec2f v2 = p2.position - p2.prevPosition;
    const Vec2f relV = v2 - v1;

    const float relVelN = relV.x * norm.x + relV.y * norm.y;
    if (relVelN < 0) {
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

void Simulator::resolveCollisions() {
  auto [w, h] = worldSize_;
  if (broadphaseType_ == BroadphaseType::UniformGrid) {
    for (Particle& par : particles_) {
      applyWall(par, w, h);
    }
    spatialGridBroadphase();
  } else if (broadphaseType_ == BroadphaseType::Qtree) {
    for (Particle& par : particles_) {
      applyWall(par, w, h);
    }
    qtreeBroadphase(16);
  } else {
    for (Particle& par : particles_) {
      applyWall(par, w, h);
    }
    naiveBroadphase();
  }
}
