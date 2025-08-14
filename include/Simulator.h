#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <vector>

struct Simulator {
	sf::Vector2f windowDims;
	std::vector<Particle> particles;

	Simulator(sf::Vector2u dims) : windowDims(dims) {};
	void spawnParticle(sf::Vector2i pos);
};

#endif
