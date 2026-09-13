#ifndef RPENGINE_COLLISION_AABB_HPP
#define RPENGINE_COLLISION_AABB_HPP

#include <math/Types.hpp>

struct AABB {
  Vec3f min{0.0f, 0.0f, 0.0f};
  Vec3f max{0.0f, 0.0f, 0.0f};

  AABB(Vec3f mn, Vec3f mx) noexcept : min(mn), max(mx) {}
};

#endif
