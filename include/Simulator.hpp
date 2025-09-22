#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.hpp>
#include <dsa/QuadTree.hpp>
#include <dsa/SpatialGrid.hpp>
#include <dsa/Vec2.hpp>
#include <random>
#include <vector>

enum class IntegrationType { Euler, Verlet };

class Simulator {
 public:
  float gravity;
  float restitution;

  Simulator(Vec2f dims, float maxParticleRadius, float g, float C_r, float dt,
            IntegrationType integrationType, int reserveParticles = 100000);

  void setWorldSize(Vec2f size) noexcept { worldSize_ = size; };
  Vec2f worldSize() const noexcept { return worldSize_; };
  void setDeltaTime(float dt) noexcept { dt_ = dt; };
  float maxParticleRadius() const noexcept { return maxParticleRadius_; };

  void spawnParticle(Vec2f pos, Vec2f vel, float r = 10.0f,
                     float m = 1.0f) noexcept;
  void update() noexcept;
  const std::vector<Particle>& getParticles() const noexcept {
    return particles_;
  };

 private:
  std::mt19937 gen_;
  Vec2f worldSize_;
  float maxParticleRadius_;
  std::vector<Particle> particles_;
  float dt_;
  IntegrationType integrationType_;

  // broad-phase
  void naiveCollisions();
  void qtreeCollisions(size_t bucketSize = 4);
  void spatialCollisions();  // TODO

  // collisions
  void applyWall(Particle& p, float w, float h);
  void particleCollision(Particle& p1, Particle& p2);
  void resolveCollisions();
};

#endif
