#ifndef RPENGINE_COLLISION_RAYCAST_HPP
#define RPENGINE_COLLISION_RAYCAST_HPP

#include <collision/Shapes.hpp>
#include <core/BodyHandle.hpp>
#include <limits>
#include <math/Transform.hpp>
#include <math/Types.hpp>

struct Ray {
  Vec3f origin{0.0f, 0.0f, 0.0f};
  Vec3f direction{0.0f, 0.0f, -1.0f};
};

struct RayHit {
  bool hit = false;
  float distance = 0.0f;
  Vec3f point{0.0f, 0.0f, 0.0f};
  Vec3f normal{0.0f, 0.0f, 0.0f};
};

struct WorldRayHit {
  BodyHandle body{};
  float distance = 0.0f;
  Vec3f point{0.0f, 0.0f, 0.0f};
  Vec3f normal{0.0f, 0.0f, 0.0f};
};

RayHit raycastShape(
    const ShapeVariant& shape, const Transform& transform, const Ray& ray,
    float maxDistance = std::numeric_limits<float>::infinity()) noexcept;

bool rayIntersectsAABB(const Ray& ray, const AABB& aabb, float maxDistance,
                       float* tEnter = nullptr,
                       float* tExit = nullptr) noexcept;

#endif
