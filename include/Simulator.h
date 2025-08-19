#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <vector>

struct Simulator {
  sf::Vector2f windowDims;
  std::vector<Particle>
      particles; // TODO maybe change data structure holding these
  float gravity;
  float restitution;
  float dt;

  Simulator(sf::Vector2u dims, float g, float C_r, float dt)
      : windowDims(dims), gravity(g), restitution(C_r), dt(dt) {};
  void spawnParticle(sf::Vector2i pos);
  void update();
};

#endif
