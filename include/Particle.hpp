#ifndef PARTICLE_H
#define PARTICLE_H

#include <cstdint>
#include <dsa/Vec2.hpp>

struct Particle {
 public:
  Vec2f position;
  Vec2f velocity;
  Vec2f acceleration;
  float radius;
  float mass;

  uint32_t id;

  Particle(Vec2f pos, Vec2f vel, float r = 10.0f, float m = 1.0f)
      : position(pos), velocity(vel), radius(r), mass(m), id(nextId()) {};

  void update(float dt) {
    velocity += acceleration * dt;
    position += velocity * dt;
  };

  void accelerate(Vec2f accel) { acceleration += accel; };

 private:
  static uint32_t nextId() {
    static uint32_t counter = 0;
    return counter++;
  }
};

#endif
