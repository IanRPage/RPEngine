#include <Particle.hpp>

Particle::Particle(Vec2f pos, Vec2f vel, float dt, float r, float m)
    : position(pos),
      prevPosition(pos - vel * dt),
      velocity(vel),
      radius(r),
      mass(m),
      id(nextId()) {
  if (mass == 0.0f) {
    invMass = 0.0f;
  } else {
    invMass = 1.0f / mass;
  }
}

void Particle::integrateEuler(float dt) noexcept {
  velocity += acceleration * dt;
  position += velocity * dt;
  acceleration = {0.0f, 0.0f};
}

void Particle::integrateVerlet(float dt) noexcept {
  Vec2f newPos = position + (position - prevPosition) + acceleration * dt * dt;
  prevPosition = position;
  position = newPos;
  acceleration = {0.0f, 0.0f};
}

void Particle::accelerate(Vec2f accel) noexcept { acceleration += accel; }

uint32_t Particle::nextId() noexcept {
  static uint32_t counter = 0;
  return counter++;
}
