#include <Particle.h>

void Particle::update(float dt) {
	velocity += acceleration * dt;
	position += velocity * dt;
	shape.setPosition(position);
}
