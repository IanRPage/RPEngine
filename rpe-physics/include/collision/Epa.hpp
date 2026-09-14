#ifndef RPENGINE_COLLISION_EPA_HPP
#define RPENGINE_COLLISION_EPA_HPP

#include <collision/Gjk.hpp>
#include <collision/Shapes.hpp>
#include <math/Transform.hpp>
#include <math/Types.hpp>

struct EpaResult {
  float penetrationDepth = 0.0f;
  Vec3f normal{0.0f, 0.0f, 0.0f};
  Vec3f contactPointOnA{0.0f, 0.0f, 0.0f};
  Vec3f contactPointOnB{0.0f, 0.0f, 0.0f};
};

EpaResult epaPenetration(const ShapeVariant& shapeA, const Transform& ta,
                         const ShapeVariant& shapeB, const Transform& tb,
                         const GjkResult& terminalSimplex) noexcept;

#endif
