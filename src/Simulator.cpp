#include <Simulator.hpp>

Simulator::Simulator(Vec2f dims, float g, float C_r, float dt,
                     IntegrationType integrationType, ResolverType resolverType,
                     int reserveParticles)
    : gravity(g),
      restitution(C_r),
      worldSize_(dims),
      dt_(dt),
      integrationType_(integrationType),
      resolverType_(resolverType) {
  std::random_device rd;
  gen_.seed(rd());
  particles_.reserve(reserveParticles);
};

void Simulator::spawnParticle(Vec2f pos, Vec2f vel, float r, float m) {
  const float vMag = vel.x * vel.x + vel.y + vel.y;
  if (!vMag) {
    std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
    vel = {dist(gen_), dist(gen_)};
  }
  particles_.emplace_back(pos, vel, r, m);
};

void Simulator::update() {
  for (Particle& par : particles_) {
    par.acceleration = {0.0f, 0.0f};
    par.accelerate({0.0f, gravity});
    par.update(dt_);
  }
  resolveCollisions();
}

void Simulator::wallCollisions() {
  auto [w, h] = worldSize_;
  for (Particle& par : particles_) {
    const float r = par.radius;

    // top/bot
    if (par.position.y < r) {
      par.position.y = r;
      par.velocity.y = -par.velocity.y * restitution;
    } else if (par.position.y > h - r) {
      par.position.y = h - r;
      par.velocity.y = -par.velocity.y * restitution;
    }

    // left/right
    if (par.position.x < r) {
      par.position.x = r;
      par.velocity.x = -par.velocity.x * restitution;
    } else if (par.position.x > w - r) {
      par.position.x = w - r;
      par.velocity.x = -par.velocity.x * restitution;
    }
  }
}

void Simulator::particleCollision(Particle& p1, Particle& p2) {
  const Vec2f d = p2.position - p1.position;
  const float d2 = d.x * d.x + d.y * d.y;
  const float sum_r = p1.radius + p2.radius;
  const float sum_r2 = sum_r * sum_r;

  if (d2 >= sum_r2 || d2 == 0.0f)
    return;

  const float invDist = 1.0f / std::sqrt(d2);
  const float dist = 1.0f / invDist;
  const Vec2f norm = d * invDist;
  const float penetration = sum_r - dist;
  const float invMassSum = 1.0f / p1.mass + 1.0 / p2.mass;

  if (penetration > 0.0f && invMassSum > 0.0f) {
    const Vec2f correction = norm * (penetration / invMassSum);
    p1.position -= correction * (1.0f / p1.mass);
    p2.position += correction * (1.0f / p2.mass);
  }

  const Vec2f relV = (p2.velocity - p1.velocity);
  const float relVel = relV.x * norm.x + relV.y * norm.y;
  if (relVel < 0) {
    const float magJ = (1.0f + restitution) * relVel / invMassSum;
    const Vec2f J = norm * magJ;
    p1.velocity += J / p1.mass;
    p2.velocity -= J / p2.mass;
  }
}

// O(n^2)
void Simulator::naiveCollisions() {
  for (size_t i = 0; i < particles_.size(); i++) {
    for (size_t j = i + 1; j < particles_.size(); j++) {
      particleCollision(particles_[i], particles_[j]);
    }
  }
}

// O(nlog(n))
void Simulator::qtreeCollisions(size_t bucketSize) {
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

void Simulator::resolveCollisions() {
  wallCollisions();
  // naiveCollisions();
  qtreeCollisions(16);
}
