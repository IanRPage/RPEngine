#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <CircleRB.h>
#include <SFML/Graphics.hpp>
#include <vector>

struct Simulator {
	sf::Vector2f windowDims;
	std::vector<CircleRB> particles;

	Simulator(sf::Vector2u dims) : windowDims(dims) {};
	void spawnParticle(sf::Vector2i pos);
};

#endif
