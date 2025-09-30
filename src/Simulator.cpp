#include <Simulator.hpp>
#include <algorithm>
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
};

void Simulator::spawnParticle(Vec2f pos, Vec2f vel, float r, float m) noexcept {
  if (particles_.size() >= capacity_) return;
  const float vn = vel.x * vel.x + vel.y + vel.y;
  if (!vn) {
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    vel = {dist(gen_), dist(gen_)};
  }
  particles_.emplace_back(pos, vel, dt_, r, m);
};

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

void Simulator::spatialGridBroadphase() {
  float cellSize = 2.0f * maxParticleRadius_;
  float invCellSize = 1.0f / cellSize;  // no divisions w/ cellSize reciprocal

  int cols = static_cast<int>(worldSize_.x * invCellSize);
  int rows = static_cast<int>(worldSize_.y * invCellSize);
  int nCells = cols * rows;

  static std::vector<int> head_;  // size should be number of cells
  static std::vector<int> next_;  // size should be number of particles
  static size_t headSize = 0;
  static size_t nextSize = 0;

  // reset head_ each frame. resize if needed
  if (headSize != static_cast<size_t>(nCells)) {
    head_.assign(nCells, -1);
    headSize = nCells;
  } else {
    std::fill(head_.begin(), head_.end(), -1);
  }

  // resize next_ if size isn't same as number of particles
  if (nextSize != particles_.size()) {
    nextSize = particles_.size();
    next_.resize(nextSize);
  }

  // create particle cell coordinates cache
  struct CellCoord {
    int x, y;
  };

  static std::vector<CellCoord> cellCoords_;
  static size_t coordSize_ = 0;

  if (coordSize_ != particles_.size()) {
    coordSize_ = particles_.size();
    cellCoords_.resize(coordSize_);
  }

  // build linked-list representation
  for (size_t i = 0; i < particles_.size(); i++) {
    const Vec2f& pos = particles_[i].position;
    int cx = static_cast<int>(pos.x * invCellSize);
    int cy = static_cast<int>(pos.y * invCellSize);
    cx = std::clamp(cx, 0, cols - 1);
    cy = std::clamp(cy, 0, rows - 1);

    cellCoords_[i] = {cx, cy};

    // c maps grid coords to flat index in head
    const int c = cy * cols + cx;

    // push_front operation
    next_[i] = head_[c];
    head_[c] = i;
  }

  // broad-phase
  for (size_t i = 0; i < particles_.size(); i++) {
    Particle& p1 = particles_[i];
    const CellCoord& cell = cellCoords_[i];
    const int cx = cell.x;
    const int cy = cell.y;

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
        const int cn = ny * cols + nx;

        // get the second particle
        for (int idx = head_[cn]; idx != -1; idx = next_[idx]) {
          // prune redundant checks
          if (idx <= static_cast<int>(i)) continue;

          Particle& p2 = particles_[idx];
          particleCollision(p1, p2);
        }
      }
    }
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

  const float invDist = 1.0f / std::sqrt(d2);
  const float dist = 1.0f / invDist;
  const Vec2f norm = d * invDist;
  const float penetration = sum_r - dist;

  const float invMassSum = p1.invMass + p2.invMass;

  if (penetration > 0.0f && invMassSum > 0.0f) {
    const Vec2f correction = norm * (penetration / invMassSum);
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
  for (Particle& par : particles_) {
    applyWall(par, w, h);
  }
  // qtreeBroadphase(16);
  spatialGridBroadphase();
}
