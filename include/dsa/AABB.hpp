#ifndef AABB_H
#define AABB_H

#include <dsa/Vec2.hpp>
#include <type_traits>

template <typename T>
struct AABB {
 public:
  static_assert(std::is_arithmetic<T>::value, "AABB scalar must be numeric");
  Vec2<T> min{}, max{};

  constexpr AABB(const Vec2<T>& mn, const Vec2<T>& size) noexcept
      : min(mn), max(mn.x + size.x, mn.y + size.y) {};

  constexpr T width() const noexcept { return max.x - min.x; };
  constexpr T height() const noexcept { return max.y - min.y; };

  constexpr bool contains(const Vec2<T>& pt) const noexcept {
    return min.x <= pt.x && max.x >= pt.x && min.y <= pt.y && max.y >= pt.y;
  };

  constexpr bool intersects(const AABB& o) const noexcept {
    return !(o.min.x > max.x || o.max.x < min.x || o.min.y > max.y ||
             o.max.y < min.y);
  };
};

using AABBf = AABB<float>;
using AABBi = AABB<int>;

#endif
