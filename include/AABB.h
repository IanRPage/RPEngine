#ifndef AABB_H
#define AABB_H

#include <type_traits>

template <typename T> struct Vec2 {
  T x{}, y{};
  constexpr Vec2() = default;
  constexpr Vec2(T x_, T y_) : x(x_), y(y_) {};

  // op overloads
  constexpr Vec2 operator*(T val) const noexcept { return {x * val, y * val}; };
};

template <typename T> struct AABB {
  static_assert(std::is_arithmetic<T>::value, "AABB scalar must be numeric");
  Vec2<T> min{}, max{};

  constexpr AABB(const Vec2<T> &mn, const Vec2<T> &size)
      : min(mn), max(mn.x + size.x, mn.y + size.y) {};

  constexpr T width() const noexcept { return max.x - min.x; };
  constexpr T height() const noexcept { return max.y - min.y; };

  constexpr bool contains(Vec2<T> &pt) const noexcept {
    return min.x <= pt.x && max.x >= pt.x && min.y <= pt.y && max.y >= pt.y;
  };

  constexpr bool intersects(const AABB &o) const noexcept {
    return !(o.min.x > max.x || o.max.x < min.x || o.min.y > max.y ||
             o.max.y < min.y);
  };
};

using Vec2f = Vec2<float>;
using AABBf = AABB<float>;
using Vec2i = Vec2<int>;
using AABBi = AABB<int>;

#endif
