#ifndef PARTICLE_H
#define PARTICLE_H

#include <dsa/Vec2.hpp>

struct Particle {
  Vec2f position;
  Vec2f velocity;
  Vec2f acceleration;
  float radius;
  float mass;

  Particle(Vec2f pos, Vec2f vel, float r = 10.0f, float m = 1.0f)
      : position(pos), velocity(vel), radius(r), mass(m) {};

  void update(float dt) {
    velocity += acceleration * dt;
    position += velocity * dt;
  };

  void accelerate(Vec2f accel) { acceleration += accel; };
};

#endif
