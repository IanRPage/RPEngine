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
           float m = 1.0f);

  void integrateEuler(float dt) noexcept;
  void integrateVerlet(float dt) noexcept;
  void accelerate(Vec2f accel) noexcept;

 private:
  static uint32_t nextId() noexcept;
};

#endif
