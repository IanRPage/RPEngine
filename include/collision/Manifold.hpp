#ifndef RPENGINE_COLLISION_MANIFOLD_HPP
#define RPENGINE_COLLISION_MANIFOLD_HPP

#include <array>
#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <collision/Shapes.hpp>
#include <core/BodyHandle.hpp>
#include <cstdint>
#include <math/Transform.hpp>
#include <math/Types.hpp>

struct ManifoldPoint {
  Vec3f localAnchorA;  // contact point in bodyA's local space, at
                       // manifold-creation time
  Vec3f localAnchorB;  // contact point in bodyB's local space
  float penetration;
  float normalImpulse{0.0f};            // warm-start accumulator
  float tangentImpulse[2]{0.0f, 0.0f};  // warm-start accumulator; idx 1
                                        // unused in 2D
};

struct Manifold {
  BodyHandle bodyA, bodyB;
  Vec3f normal;
  std::array<ManifoldPoint, 4> points;
  uint8_t pointCount{0};
};

Manifold buildManifold(const ShapeVariant& shapeA, const Transform& ta,
                       const ShapeVariant& shapeB, const Transform& tb,
                       BodyHandle bodyA, BodyHandle bodyB,
                       const GjkResult& terminalSimplex,
                       const EpaResult& epa) noexcept;

#endif
