#ifndef CIRCLE_H
#define CIRCLE_H
#include <Rigidbody.h>
#include <SFML/Graphics.hpp>

struct Circle : sf::CircleShape, Rigidbody {
  sf::Vector2f position;
  sf::Vector2f velocity;
  sf::Vector2f acceleration;
  float mass;

  Circle(sf::Vector2f pos, sf::Vector2f vel, float r = 10.0f, float m = 1.0f,
         sf::Color color = sf::Color::Red)
      : position(pos), velocity(vel), sf::CircleShape(r), mass(m) {
    setFillColor(color);
    setPosition(position);
  };

  void update(float dt) override;
  void applyForce(sf::Vector2f force) override;
  sf::Vector2f getPosition() const override { return position; };
  sf::Vector2f getVelocity() const override { return velocity; };
  float getMass() const override { return mass; };
};
#endif
