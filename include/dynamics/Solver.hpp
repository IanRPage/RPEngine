#ifndef RPENGINE_DYNAMICS_SOLVER_HPP
#define RPENGINE_DYNAMICS_SOLVER_HPP

#include <collision/Manifold.hpp>
#include <core/BodyStore.hpp>
#include <span>
#include <vector>

struct SolverConfig {
  int velocityIterations = 8;
  int positionIterations = 4;
  float slop = 0.005f;                     // allowed penetration, avoids jitter
  float maxCorrectionPerIteration = 0.2f;  // clamps single position-solve step
  float baumgartePositionFactor = 0.2f;
};

std::vector<float> prepareRestitutionBias(std::span<const Manifold> manifolds,
                                          const BodyStore& bodies) noexcept;

void warmStart(std::span<Manifold> manifolds, BodyStore& bodies) noexcept;

void solveVelocity(std::span<Manifold> manifolds, BodyStore& bodies,
                   std::span<const float> restitutionBias) noexcept;

void solvePosition(std::span<Manifold> manifolds, BodyStore& bodies,
                   const SolverConfig& config) noexcept;

#endif
