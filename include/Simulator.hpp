#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Particle.hpp>
#include <dsa/QuadTree.hpp>
#include <dsa/SpatialGrid.hpp>
#include <dsa/Vec2.hpp>
#include <random>
#include <vector>

enum class IntegrationType { Euler, Verlet };
enum class BroadphaseType { Naive, Qtree, UniformGrid };

class Simulator {
 public:
  float gravity;
  float restitution;

  Simulator(Vec2f dims, float maxParticleRadius, float g, float C_r, float dt,
            IntegrationType integrationType, BroadphaseType broadphaseType,
            size_t maxParticles = 100000);

  void configure(Vec2f size, float dt = 1.0f / 60.0f);
  void setWorldSize(Vec2f size) noexcept { worldSize_ = size; }
  Vec2f worldSize() const noexcept { return worldSize_; }
  void setDeltaTime(float dt) noexcept { dt_ = dt; }
  float maxParticleRadius() const noexcept { return maxParticleRadius_; }

  void spawnParticle(Vec2f pos, Vec2f vel, float r = 10.0f,
                     float m = 1.0f) noexcept;
  void update() noexcept;
  const std::vector<Particle>& particles() const noexcept { return particles_; }
  size_t capacity() const noexcept { return capacity_; }
  void setIntegrationType(IntegrationType integrationType) noexcept {
    integrationType_ = integrationType;
  }
  void setBroadphaseType(BroadphaseType broadphaseType) noexcept {
    broadphaseType_ = broadphaseType;
  }

  void radialPush(const Vec2f& origin, const float radius,
                  const float mag = 1000.0f, const int scale = 1);
  // void radialPush(const Vec2f& origin, const float radius = 100.0f,
  //                 const float mag = 1000.0f);

 private:
  std::mt19937 gen_;
  Vec2f worldSize_;
  float maxParticleRadius_;
  std::vector<Particle> particles_;
  float dt_;
  IntegrationType integrationType_;
  BroadphaseType broadphaseType_;

  SpatialGrid spatialGrid_;
  size_t capacity_;

  // broad-phase
  void naiveBroadphase();
  void qtreeBroadphase(size_t bucketSize = 4);
  void spatialGridBroadphase();

  // collisions
  void applyWall(Particle& p, float w, float h);
  void particleCollision(Particle& p1, Particle& p2);
  void resolveCollisions();
};

#endif
