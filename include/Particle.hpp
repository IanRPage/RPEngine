#ifndef PARTICLE_H
#define PARTICLE_H

#include <cstdint>
#include <dsa/Vec2.hpp>

struct Particle {
 public:
  Vec2f position;
  Vec2f prevPosition;
  Vec2f velocity;
  Vec2f acceleration;
  float radius;
  float mass;
  float invMass;
  uint32_t id;

  Particle(Vec2f pos, Vec2f vel, float dt = 1.0f / 60.0f, float r = 10.0f,
           float m = 1.0f)
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
  };

  void integrateEuler(float dt) noexcept {
    velocity += acceleration * dt;
    position += velocity * dt;
    acceleration = {0.0f, 0.0f};
  }

  void integrateVerlet(float dt) noexcept {
    Vec2f newPos =
        position + (position - prevPosition) + acceleration * dt * dt;
    prevPosition = position;
    position = newPos;
    acceleration = {0.0f, 0.0f};
  }

  void accelerate(Vec2f accel) noexcept { acceleration += accel; };

 private:
  static uint32_t nextId() noexcept {
    static uint32_t counter = 0;
    return counter++;
  }
};

#endif
