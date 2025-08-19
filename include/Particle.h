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

  Particle(sf::Vector2f pos, sf::Vector2f vel, float m = 1.0f, float r = 10.0f)
      : position(pos), velocity(vel), mass(m), radius(r), shape(r) {
    shape.setPosition(pos);
    shape.setFillColor(sf::Color::White); // TODO change this for varying colors
  };

	void update(float dt);
  void accelerate(sf::Vector2f accel) { acceleration += accel; };
};

#endif
