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
      numParticles_(0),
      capacity_(maxParticles) {
  std::random_device rd;
  gen_.seed(rd());

  // SoA
  positions_.x.reserve(maxParticles);
  positions_.y.reserve(maxParticles);
  prevPositions_.x.reserve(maxParticles);
  prevPositions_.y.reserve(maxParticles);
  velocities_.x.reserve(maxParticles);
  velocities_.y.reserve(maxParticles);
  accelerations_.x.reserve(maxParticles);
  accelerations_.y.reserve(maxParticles);

  spatialGrid_.configure(2.0f * maxParticleRadius_, worldSize_);
};

void Simulator::configure(Vec2f size, float dt) {
  worldSize_ = size;
  dt_ = dt;
  spatialGrid_.configure(2.0f * maxParticleRadius_, worldSize_);
}

void Simulator::spawnParticle(Vec2f pos, Vec2f vel) noexcept {
  if (numParticles_ >= capacity_) return;
  const float vn = vel.x * vel.x + vel.y + vel.y;
  if (!vn) {
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    vel = {dist(gen_), dist(gen_)};
  }

  positions_.x.push_back(pos.x);
  positions_.y.push_back(pos.y);
  prevPositions_.x.push_back(pos.x - vel.x * dt_);
  prevPositions_.y.push_back(pos.y - vel.y * dt_);
  velocities_.x.push_back(vel.x);
  velocities_.y.push_back(vel.y);
  accelerations_.x.push_back(0.0f);
  accelerations_.y.push_back(0.0f);
  numParticles_++;
};

void Simulator::radialPush(const Vec2f& origin, const float radius,
                           const float mag, const int scale) {
  spatialGrid_.queryDoSomething(
      -1, origin,
      [&](int neiIdx) {
        const Vec2f pos = {positions_.x[neiIdx], positions_.y[neiIdx]};

        const Vec2f d = pos - origin;
        const float d2 = d.x * d.x + d.y * d.y;

        if (d2 > radius * radius) return;

        const float invDist = 1.0f / std::sqrt(d2);
        const Vec2f norm = d * invDist;

        accelerations_.x[neiIdx] += norm.x * mag;
        accelerations_.y[neiIdx] += norm.y * mag;
      },
      scale);
}

void Simulator::update() noexcept {
  if (integrationType_ == IntegrationType::Euler) {
    for (size_t i = 0; i < numParticles_; i++) {
      accelerations_.x[i] += 0.0f;
      accelerations_.y[i] += gravity;
      integrateEuler(i, dt_);
    }
  } else {
    for (size_t i = 0; i < numParticles_; i++) {
      accelerations_.x[i] += 0.0f;
      accelerations_.y[i] += gravity;
      integrateVerlet(i, dt_);
    }
  }
  resolveCollisions();
}

// O(n^2)
void Simulator::naiveBroadphase() {
  for (size_t i = 0; i < numParticles_; i++) {
    for (size_t j = i + 1; j < numParticles_; j++) {
      particleCollision(i, j);
    }
  }
}

// O(nlog(n))
void Simulator::qtreeBroadphase(size_t bucketSize) {
  QuadTree qtree(AABBf({0.0f, 0.0f}, {worldSize_.x, worldSize_.y}), bucketSize);
  for (size_t i = 0; i < numParticles_; i++) {
    const Vec2f pos = {positions_.x[i], positions_.y[i]};
    qtree.insert(i, pos);
  }

  for (size_t i = 0; i < numParticles_; i++) {
    const Vec2f c1 = {positions_.x[i], positions_.y[i]};
    const float r1 = maxParticleRadius_;  // address for variable radius
    const AABBf queryRange({c1.x - 2.0f * r1, c1.y - 2.0f * r1},
                           {4.0f * r1, 4.0f * r1});

    std::vector<size_t> neighbors;
    qtree.query(neighbors, queryRange, [this](size_t idx) { return getPos(idx); });

    for (size_t j : neighbors) {
      if (i >= j) continue;
      particleCollision(i, j);
    }
  }
}

// O(n)
void Simulator::spatialGridBroadphase() {
  spatialGrid_.resize(numParticles_);
  spatialGrid_.build(positions_, numParticles_);

  for (size_t i = 0; i < numParticles_; i++) {
    const Vec2f pos = {positions_.x[i], positions_.y[i]};
    spatialGrid_.queryDoSomething(
        i, pos, [&](int neiIdx) { particleCollision(i, neiIdx); });
  }
}

void Simulator::applyWall(size_t i) {
  const float r = maxParticleRadius_;
  auto [w, h] = worldSize_;

  // current position
  float& x = positions_.x[i];
  float& y = positions_.y[i];

  // prev position
  float& xPrev = prevPositions_.x[i];
  float& yPrev = prevPositions_.y[i];

  // current velocity
  float& vx = velocities_.x[i];
  float& vy = velocities_.y[i];

  if (integrationType_ == IntegrationType::Euler) {
    // top/bot
    if (y < r) {
      y = r;
      vy = -vy * restitution;
    } else if (y > h - r) {
      y = h - r;
      vy = -vy * restitution;
    }

    // left/right
    if (x < r) {
      x = r;
      vx = -vx * restitution;
    } else if (x > w - r) {
      x = w - r;
      vx = -vx * restitution;
    }
  } else {
    float vx = x - xPrev;
    float vy = y - yPrev;

    // top/bot
    if (y < r) {
      y = r;
      yPrev = y + vy * restitution;
    } else if (y > h - r) {
      y = h - r;
      yPrev = y + vy * restitution;
    }

    // left/right
    if (x < r) {
      x = r;
      xPrev = x + vx * restitution;
    } else if (x > w - r) {
      x = w - r;
      xPrev = x + vx * restitution;
    }
  }
}

void Simulator::particleCollision(size_t i, size_t j) {
  // current positions
  float& p1x = positions_.x[i];
  float& p1y = positions_.y[i];
  float& p2x = positions_.x[j];
  float& p2y = positions_.y[j];

  // prev positions
  float& p1xPrev = prevPositions_.x[i];
  float& p1yPrev = prevPositions_.y[i];
  float& p2xPrev = prevPositions_.x[j];
  float& p2yPrev = prevPositions_.y[j];

  // current velocities
  float& p1vx = velocities_.x[i];
  float& p1vy = velocities_.y[i];
  float& p2vx = velocities_.x[j];
  float& p2vy = velocities_.y[j];

  const Vec2f d = {p2x - p1x, p2y - p1y};
  const float d2 = d.x * d.x + d.y * d.y;
  const float sum_r = 2.0f * maxParticleRadius_;  // address for variable radius
  const float sum_r2 = sum_r * sum_r;             // address for variable radius

  if (d2 >= sum_r2) return;

  // if small dist apart
  if (d2 < 1e-12f) {
    Vec2f n = {1.0f, 0.0f};
    const float half = sum_r * 0.5f;  // address for variable radius & mass

    const float invMassSum = 2.0f;  // address for variable radius & mass
    if (invMassSum > 0.0f) {
      p1xPrev = p1x;
      p1yPrev = p1y;
      p2xPrev = p2x;
      p2yPrev = p2y;

      // address for variable radius & mass
      p1x -= n.x * half * 0.5f;
      p1y -= n.y * half * 0.5f;
      p2x += n.x * half * 0.5f;
      p2y += n.y * half * 0.5f;
    }
    return;
  }

  const float invDist = 1.0f / std::sqrt(d2);
  const float dist = 1.0f / invDist;
  const Vec2f norm = d * invDist;
  const float penetration = sum_r - dist;
  const float invMassSum = 2;  // address for variable mass (currently 1)

  if (penetration > 0.0f && invMassSum > 0.0f) {
    float percent = 0.30f;
    const Vec2f correction = norm * (percent * penetration / invMassSum);

    p1x -= correction.x;  // address for variable mass
    p1y -= correction.y;  // address for variable mass
    p2x += correction.x;  // address for variable mass
    p2y += correction.y;  // address for variable mass
  }

  if (integrationType_ == IntegrationType::Euler) {
    const Vec2f relV = {p2x - p1x, p2y - p1y};
    const float relVel = relV.x * norm.x + relV.y * norm.y;
    if (relVel < 0) {
      const float magJ = (1.0f + restitution) * relVel / invMassSum;
      const Vec2f J = norm * magJ;

      // address for variable mass
      p1vx += J.x;
      p1vy += J.y;
      p2vx -= J.x;
      p2vy -= J.y;
    }
  } else {
    Vec2f v1 = {p1x - p1xPrev, p1y - p1yPrev};
    Vec2f v2 = {p2x - p2xPrev, p2y - p2yPrev};
    const Vec2f relV = v2 - v1;

    const float relVelN = relV.x * norm.x + relV.y * norm.y;
    if (relVelN < 0) {
      // address 1.0f for variable mass
      const float w1 = 1.0f / invMassSum;
      const float w2 = 1.0f / invMassSum;

      const float nRelVelN = -restitution * relVelN;
      const float dRelVelN = nRelVelN - relVelN;

      const Vec2f dV = norm * dRelVelN;
      v1 -= dV * w1;
      v2 += dV * w2;

      p1xPrev = p1x - v1.x;
      p1yPrev = p1y - v1.y;
      p2xPrev = p2x - v1.x;
      p2yPrev = p2y - v1.y;
    }
  }
}

void Simulator::resolveCollisions() {
  if (broadphaseType_ == BroadphaseType::UniformGrid) {
    for (size_t i = 0; i < numParticles_; i++) {
      applyWall(i);
    }
    spatialGridBroadphase();
  } else if (broadphaseType_ == BroadphaseType::Qtree) {
    for (size_t i = 0; i < numParticles_; i++) {
      applyWall(i);
    }
    qtreeBroadphase(16);
  } else {
    for (size_t i = 0; i < numParticles_; i++) {
      applyWall(i);
    }
    naiveBroadphase();
  }
}

void Simulator::integrateEuler(size_t i, float dt) {
  // update velocity
  velocities_.x[i] += accelerations_.x[i] * dt;
  velocities_.y[i] += accelerations_.y[i] * dt;

  // update position
  positions_.x[i] += velocities_.x[i] * dt;
  positions_.y[i] += velocities_.y[i] * dt;

  // reset acceleration
  accelerations_.x[i] = 0.0f;
  accelerations_.y[i] = 0.0f;
}

void Simulator::integrateVerlet(size_t i, float dt) {
  float& x = positions_.x[i];
  float& y = positions_.y[i];
  float& prevX = prevPositions_.x[i];
  float& prevY = prevPositions_.y[i];
  float& ax = accelerations_.x[i];
  float& ay = accelerations_.y[i];

  const float nx = x + (x - prevX) + ax * dt * dt;
  const float ny = y + (y - prevY) + ay * dt * dt;

  prevX = x;
  prevY = y;
  x = nx;
  y = ny;
  ax = ay = 0.0f;
}
