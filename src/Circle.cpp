#include <Circle.h>

void Circle::update(float dt) {
	velocity += acceleration * dt;
	position += velocity * dt;
	setPosition(position);
	acceleration = sf::Vector2f(0.0f, 0.0f);
}

void Circle::applyForce(sf::Vector2f force) {
	acceleration += force / mass;
}
