#ifndef VEC2_H
#define VEC2_H

template <typename T> struct Vec2 {
  T x{}, y{};
  constexpr Vec2() = default;
  constexpr Vec2(T x_, T y_) : x(x_), y(y_) {};

  // op overloads
  constexpr Vec2 operator*(T val) const noexcept { return {x * val, y * val}; };
};

using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;

#endif
