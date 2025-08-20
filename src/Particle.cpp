#include <Particle.h>

void Particle::update(float dt) {
	velocity += acceleration * dt;
	position += velocity * dt;
	shape.setPosition({position.x + radius, position.y + radius});
}

void Particle::drawPos(sf::RenderWindow &window) const {
	sf::VertexArray v(sf::PrimitiveType::Points, 1);
	v[0].position = shape.getPosition();
	window.draw(v);
}
