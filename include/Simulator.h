#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <random>
#include <vector>

class Simulator {
private:
  std::mt19937 gen;
  sf::Vector2f windowDims;
  std::vector<Particle> particles;
  float dt;
  void wallCollisions();
  void particleCollisions();
  void handleCollisions();

public:
  float gravity;
  float restitution;

  Simulator(sf::Vector2u dims, float g, float C_r, float dt);
  void spawnParticle(sf::Vector2i pos, sf::Texture *texture = nullptr);
  void update();
  const std::vector<Particle> &getParticles() const { return particles; };
};

#endif
