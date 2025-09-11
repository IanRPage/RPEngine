#ifndef VEC2_H
#define VEC2_H

template <typename T> struct Vec2 {
  T x{}, y{};

  constexpr Vec2() = default;
  constexpr Vec2(T x_, T y_) : x(x_), y(y_) {};

  // op overloads
  constexpr Vec2 operator+(const Vec2 &r) const noexcept {
    return {x + r.x, y + r.y};
  };
  constexpr Vec2 operator-(const Vec2 &r) const noexcept {
    return {x - r.x, y - r.y};
  };
  constexpr Vec2 operator*(T val) const noexcept { return {x * val, y * val}; };
  constexpr Vec2 operator/(T val) const noexcept { return {x / val, y / val}; };

  constexpr Vec2 &operator+=(const Vec2 &r) noexcept {
    x += r.x;
    y += r.y;
    return *this;
  };
  constexpr Vec2 &operator-=(const Vec2 &r) noexcept {
    x -= r.x;
    y -= r.y;
    return *this;
  };
};

using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;
using Vec2u = Vec2<unsigned int>;

#endif
