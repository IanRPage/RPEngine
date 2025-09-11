#ifndef PARTICLE_H
#define PARTICLE_H

#include <SFML/Graphics.hpp>
#include <dsa/Vec2.hpp>

// TODO remove SFML from Particle

struct Particle {
  Vec2f position;
  Vec2f velocity;
  Vec2f acceleration;
  float radius;
  float mass;
  sf::CircleShape shape;

  Particle(Vec2f pos, Vec2f vel, float r = 10.0f, float m = 1.0f,
           sf::Texture *texture = nullptr)
      : position(pos), velocity(vel), radius(r), mass(m), shape(r) {
    shape.setOrigin({r, r});
    shape.setPosition({pos.x, pos.y});
    if (texture)
      shape.setTexture(texture);
  };

  void update(float dt) {
    velocity += acceleration * dt;
    position += velocity * dt;
    shape.setPosition({position.x, position.y});
  };

  void accelerate(Vec2f accel) { acceleration += accel; };
};

#endif
