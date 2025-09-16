#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.hpp>
#include <dsa/QuadTree.hpp>
#include <dsa/Vec2.hpp>
#include <random>
#include <vector>

enum class IntegrationType { Euler, Verlet };
enum class ResolverType { Impulse, Position };

class Simulator {
 public:
  float gravity;
  float restitution;

  Simulator(Vec2f dims = {100.0f, 100.0f}, float g = 100.0f, float C_r = 0.95f,
            float dt = 1.0f / 60.0f,
            IntegrationType integrationType = IntegrationType::Euler,
            ResolverType resolverType = ResolverType::Impulse,
            int reserveParticles = 100000);

  void setWorldSize(Vec2f size) { worldSize_ = size; };
  Vec2f worldSize() const { return worldSize_; };

  void spawnParticle(Vec2f pos, Vec2f vel, float r = 10.0f, float m = 1.0f);
  void update();
  const std::vector<Particle>& getParticles() const { return particles_; };

 private:
  std::mt19937 gen_;
  Vec2f worldSize_;
  std::vector<Particle> particles_;
  float dt_;
  IntegrationType integrationType_;
  ResolverType resolverType_;

  void wallCollisions();
  void particleCollision(Particle& p1, Particle& p2);
  void naiveCollisions();
  void qtreeCollisions(size_t bucketSize = 4);
  void resolveCollisions();
};

#endif
