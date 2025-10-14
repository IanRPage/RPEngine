#ifndef SIMULATOR_H
#define SIMULATOR_H

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

  // setters
  void configure(Vec2f size, float dt = 1.0f / 60.0f);
  void setDeltaTime(float dt) noexcept { dt_ = dt; }
  void setWorldSize(Vec2f size) noexcept { worldSize_ = size; }
  void update() noexcept;
  void setIntegrationType(IntegrationType integrationType) noexcept {
    integrationType_ = integrationType;
  }
  void setBroadphaseType(BroadphaseType broadphaseType) noexcept {
    broadphaseType_ = broadphaseType;
  }

  // getters
  Vec2f worldSize() const noexcept { return worldSize_; }
  float maxParticleRadius() const noexcept { return maxParticleRadius_; }
  size_t capacity() const noexcept { return capacity_; }
  size_t numParticles() const noexcept { return numParticles_; }
  Vec2f getPos(size_t i) const noexcept {
    return {positions_.x[i], positions_.y[i]};
  }

  // sim functions / helpers
  void spawnParticle(Vec2f pos, Vec2f vel) noexcept;
  void radialPush(const Vec2f& origin, const float radius,
                  const float mag = 1000.0f, const int scale = 1);

 private:
  std::mt19937 gen_;
  Vec2f worldSize_;
  float maxParticleRadius_;

  // SoA
  struct vecList2D {
    std::vector<float> x;
    std::vector<float> y;
  };

  vecList2D prevPositions_;
  vecList2D positions_;
  vecList2D velocities_;
  vecList2D accelerations_;

  float dt_;
  IntegrationType integrationType_;
  BroadphaseType broadphaseType_;

  SpatialGrid spatialGrid_;
  size_t numParticles_;
  size_t capacity_;

  // broad-phase
  void naiveBroadphase();
  void qtreeBroadphase(size_t bucketSize = 4);
  void spatialGridBroadphase();

  // collisions
  void applyWall(size_t i);
  void particleCollision(size_t i, size_t j);
  void resolveCollisions();

  // helpers
  void integrateEuler(size_t i, float dt);
  void integrateVerlet(size_t i, float dt);
};

#endif
