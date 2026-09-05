#include <gtest/gtest.h>
#include <math/Types.hpp>

// tests if the toolchain actually links and runs
TEST(MathTypesSmokeTest, BasicConstructionAndOps) {
  Vec3f a{1.0f, 2.0f, 3.0f};
  Vec3f b{4.0f, 5.0f, 6.0f};
  Vec3f sum = a + b;
  EXPECT_FLOAT_EQ(sum.x, 5.0f);
  EXPECT_FLOAT_EQ(sum.y, 7.0f);
  EXPECT_FLOAT_EQ(sum.z, 9.0f);

  Quatf identity{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3f rotated = identity * a;
  EXPECT_FLOAT_EQ(rotated.x, a.x);
  EXPECT_FLOAT_EQ(rotated.y, a.y);
  EXPECT_FLOAT_EQ(rotated.z, a.z);
}
