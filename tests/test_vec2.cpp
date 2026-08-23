#include <dsa/Vec2.hpp>
#include <gtest/gtest.h>

TEST(Vec2Test, DefaultConstructor) {
  Vec2f v;
  EXPECT_FLOAT_EQ(v.x, 0.0f);
  EXPECT_FLOAT_EQ(v.y, 0.0f);
}

TEST(Vec2Test, ParameterizedConstructor) {
  Vec2f v(3.0f, 4.0f);
  EXPECT_FLOAT_EQ(v.x, 3.0f);
  EXPECT_FLOAT_EQ(v.y, 4.0f);
}

TEST(Vec2Test, Addition) {
  Vec2f a(1.0f, 2.0f);
  Vec2f b(3.0f, 4.0f);
  Vec2f c = a + b;
  EXPECT_FLOAT_EQ(c.x, 4.0f);
  EXPECT_FLOAT_EQ(c.y, 6.0f);
}

TEST(Vec2Test, Subtraction) {
  Vec2f a(5.0f, 7.0f);
  Vec2f b(2.0f, 3.0f);
  Vec2f c = a - b;
  EXPECT_FLOAT_EQ(c.x, 3.0f);
  EXPECT_FLOAT_EQ(c.y, 4.0f);
}

TEST(Vec2Test, ScalarMultiplication) {
  Vec2f a(2.0f, 3.0f);
  Vec2f c = a * 2.0f;
  EXPECT_FLOAT_EQ(c.x, 4.0f);
  EXPECT_FLOAT_EQ(c.y, 6.0f);
}

TEST(Vec2Test, ScalarDivision) {
  Vec2f a(6.0f, 8.0f);
  Vec2f c = a / 2.0f;
  EXPECT_FLOAT_EQ(c.x, 3.0f);
  EXPECT_FLOAT_EQ(c.y, 4.0f);
}

TEST(Vec2Test, PlusEquals) {
  Vec2f a(1.0f, 2.0f);
  a += Vec2f(3.0f, 4.0f);
  EXPECT_FLOAT_EQ(a.x, 4.0f);
  EXPECT_FLOAT_EQ(a.y, 6.0f);
}

TEST(Vec2Test, MinusEquals) {
  Vec2f a(5.0f, 7.0f);
  a -= Vec2f(2.0f, 3.0f);
  EXPECT_FLOAT_EQ(a.x, 3.0f);
  EXPECT_FLOAT_EQ(a.y, 4.0f);
}

TEST(Vec2Test, IntegerType) {
  Vec2i v(1, 2);
  Vec2i w(3, 4);
  Vec2i r = v + w;
  EXPECT_EQ(r.x, 4);
  EXPECT_EQ(r.y, 6);
}
