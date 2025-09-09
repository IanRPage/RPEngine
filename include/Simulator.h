#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <dsa/QuadTree.h>
#include <dsa/Vec2.h>
#include <random>
#include <vector>

// TODO decouple SFML from Simulator

class Simulator {
private:
  std::mt19937 gen;
  Vec2f windowDims;
  std::vector<Particle> particles;
  float dt;
  void wallCollisions();
  void particleCollision(Particle &p1, Particle &p2);
  void naiveCollisions();
  void qtreeCollisions(size_t bucketSize = 4);
  void resolveCollisions();

public:
  float gravity;
  float restitution;

  Simulator(Vec2u dims, float g, float C_r, float dt, int resParticles = 1000);

	// TODO decouple SFML from here
  void spawnParticle(Vec2f pos, float r = 10.0f, float m = 1.0f,
                     sf::Texture *texture = nullptr);
  void update();
  const std::vector<Particle> &getParticles() const { return particles; };
};

#endif
