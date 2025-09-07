#ifndef PARTICLE_H
#define PARTICLE_H

#include <SFML/Graphics.hpp>

struct Particle {
  sf::Vector2f position;
  sf::Vector2f velocity;
  sf::Vector2f acceleration;
  float mass;
  float radius;
  sf::CircleShape shape;

  Particle(sf::Vector2f pos, sf::Vector2f vel, sf::Texture *texture,
           float m = 1.0f, float r = 10.0f)
      : position(pos), velocity(vel), mass(m), radius(r), shape(r) {
    shape.setOrigin({radius, radius});
    shape.setPosition(position);
    if (texture != nullptr)
      shape.setTexture(texture);
  };

  void update(float dt) {
		velocity += acceleration * dt;
		position += velocity * dt;
		shape.setPosition(position);
	};

  void accelerate(sf::Vector2f accel) { acceleration += accel; };
};

#endif
