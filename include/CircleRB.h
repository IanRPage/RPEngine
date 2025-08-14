#ifndef CIRCLERB_H
#define CIRCLERB_H
#include <SFML/Graphics.hpp>

struct CircleRB : sf::CircleShape {
  sf::Vector2f position;
  sf::Vector2f velocity;
  sf::Vector2f acceleration;
  float mass;

  CircleRB(sf::Vector2f pos, sf::Vector2f vel, float r = 10.0f,
           sf::Color color = sf::Color::Red)
      : position(pos), velocity(vel), sf::CircleShape(r) {
    setFillColor(color);
		setPosition(position);
  };
};
#endif
