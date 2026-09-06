#ifndef RPENGINE_COLLISION_SHAPES_HPP
#define RPENGINE_COLLISION_SHAPES_HPP

#include <collision/AABB.hpp>
#include <math/Constants.hpp>
#include <math/Transform.hpp>
#include <math/Types.hpp>
#include <variant>
#include <vector>

namespace shapes_detail {
inline float signNonZero(float v) noexcept { return v < 0.0f ? -1.0f : 1.0f; }
}  // namespace shapes_detail

struct SphereShape {
  float radius;

  Vec3f support(Vec3f direction) const noexcept {
    float len = glm::length(direction);
    if (len < VECTOR_LENGTH_EPSILON) return Vec3f{radius, 0.0f, 0.0f};
    return (direction / len) * radius;
  }

  float boundingRadius() const noexcept { return radius; }

  Mat3f localInertiaTensor(float mass) const noexcept;

  AABB localAABB() const noexcept {
    return AABB(Vec3f(-radius), Vec3f(radius));
  }
};

struct BoxShape {
  Vec3f halfExtents;

  Vec3f support(Vec3f direction) const noexcept {
    return Vec3f(shapes_detail::signNonZero(direction.x) * halfExtents.x,
                 shapes_detail::signNonZero(direction.y) * halfExtents.y,
                 shapes_detail::signNonZero(direction.z) * halfExtents.z);
  }

  float boundingRadius() const noexcept { return glm::length(halfExtents); }

  Mat3f localInertiaTensor(float mass) const noexcept;

  AABB localAABB() const noexcept { return AABB(-halfExtents, halfExtents); }
};

struct CapsuleShape {
  float radius;
  float halfHeight;  // segment runs along local +Y/-Y axis

  Vec3f support(Vec3f direction) const noexcept {
    float len = glm::length(direction);
    Vec3f n = (len < VECTOR_LENGTH_EPSILON) ? Vec3f(0.0f) : direction / len;
    float yOffset = direction.y >= 0.0f ? halfHeight : -halfHeight;
    return Vec3f(radius * n.x, radius * n.y + yOffset, radius * n.z);
  }

  float boundingRadius() const noexcept { return halfHeight + radius; }

  Mat3f localInertiaTensor(float mass) const noexcept;

  AABB localAABB() const noexcept {
    return AABB(Vec3f(-radius, -halfHeight - radius, -radius),
                Vec3f(radius, halfHeight + radius, radius));
  }
};

struct ConvexHullShape {
  std::vector<Vec3f>
      localVertices;  // relative to centroid; CCW winding for 2D hulls

  explicit ConvexHullShape(std::vector<Vec3f> vertices);

  Vec3f support(Vec3f direction) const noexcept;

  float boundingRadius() const noexcept { return cachedBoundingRadius_; }

  Mat3f localInertiaTensor(float mass) const noexcept;

  AABB localAABB() const noexcept {
    Vec3f mn = localVertices.front();
    Vec3f mx = localVertices.front();
    for (const Vec3f& v : localVertices) {
      mn = glm::min(mn, v);
      mx = glm::max(mx, v);
    }
    return AABB(mn, mx);
  }

 private:
  float cachedBoundingRadius_ = 0.0f;  // precomputed in constructor
};

using ShapeVariant =
    std::variant<SphereShape, BoxShape, CapsuleShape, ConvexHullShape>;

Vec3f worldSupport(const ShapeVariant& shape, const Transform& t,
                   Vec3f worldDir) noexcept;

AABB worldAABB(const ShapeVariant& shape, const Transform& t) noexcept;

bool isFlatPair(const ShapeVariant& shapeA, const Transform& ta,
                const ShapeVariant& shapeB, const Transform& tb) noexcept;

#endif
