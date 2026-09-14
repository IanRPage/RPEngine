#ifndef RPENGINE_MATH_TRANSFORM_HPP
#define RPENGINE_MATH_TRANSFORM_HPP

#include <math/Types.hpp>

struct Transform {
  Vec3f position{0.0f, 0.0f, 0.0f};
  Quatf orientation{1.0f, 0.0f, 0.0f, 0.0f};  // identity: w, x, y, z
};

Vec3f transformPoint(const Transform& t, Vec3f localPoint) noexcept;
Vec3f transformDirection(const Transform& t, Vec3f localDir) noexcept;
Transform inverse(const Transform& t) noexcept;
Transform compose(const Transform& parent, const Transform& child) noexcept;

#endif
