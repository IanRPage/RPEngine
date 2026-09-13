#ifndef RPENGINE_COLLISION_GJK_HPP
#define RPENGINE_COLLISION_GJK_HPP

#include <array>
#include <collision/Shapes.hpp>
#include <math/Transform.hpp>
#include <math/Types.hpp>

struct SupportPoint {
  Vec3f a;
  Vec3f b;
  Vec3f diff;
};

SupportPoint minkowskiSupport(const ShapeVariant& shapeA, const Transform& ta,
                              const ShapeVariant& shapeB, const Transform& tb,
                              Vec3f dir) noexcept;

struct GjkResult {
  bool overlapping = false;
  std::array<SupportPoint, 4> simplex{};
  int simplexCount = 0;
};

GjkResult gjkOverlap(const ShapeVariant& shapeA, const Transform& ta,
                     const ShapeVariant& shapeB, const Transform& tb) noexcept;

#endif
