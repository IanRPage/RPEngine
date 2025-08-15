#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <SFML/Graphics.hpp>

class Rigidbody {
public:
  virtual ~Rigidbody() = default;
  virtual void update(float dt) = 0;
  virtual void applyForce(sf::Vector2f force) = 0;
  virtual sf::Vector2f getPosition() const = 0;
  virtual sf::Vector2f getVelocity() const = 0;
  virtual float getMass() const = 0;
};

#endif
