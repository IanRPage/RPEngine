#ifndef RPENGINE_DYNAMICS_SOLVER_HPP
#define RPENGINE_DYNAMICS_SOLVER_HPP

#include <collision/Manifold.hpp>
#include <core/BodyStore.hpp>
#include <span>
#include <unordered_map>
#include <vector>

struct SolverConfig {
  int velocityIterations = 8;
  int positionIterations = 4;
  float slop = 0.005f;                     // allowed penetration, avoids jitter
  float maxCorrectionPerIteration = 0.2f;  // clamps single position-solve step
  float baumgartePositionFactor = 0.2f;
};

class InvInertiaWorldCache {
 public:
  explicit InvInertiaWorldCache(const BodyStore& bodies) noexcept
      : bodies_(&bodies) {}

  const Mat3f& get(BodyHandle h) noexcept;
  void invalidate(BodyHandle h) noexcept;

 private:
  const BodyStore* bodies_;
  std::unordered_map<uint32_t, Mat3f> cache_;
};

std::vector<float> prepareRestitutionBias(std::span<const Manifold> manifolds,
                                          const BodyStore& bodies) noexcept;

void warmStart(std::span<Manifold> manifolds, BodyStore& bodies,
               InvInertiaWorldCache& invInertiaCache) noexcept;
void warmStart(std::span<Manifold> manifolds, BodyStore& bodies) noexcept;

void solveVelocity(std::span<Manifold> manifolds, BodyStore& bodies,
                   std::span<const float> restitutionBias,
                   InvInertiaWorldCache& invInertiaCache) noexcept;
void solveVelocity(std::span<Manifold> manifolds, BodyStore& bodies,
                   std::span<const float> restitutionBias) noexcept;

void solvePosition(std::span<Manifold> manifolds, BodyStore& bodies,
                   const SolverConfig& config) noexcept;

#endif
