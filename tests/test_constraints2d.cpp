#include <gtest/gtest.h>
#include <math/Constraints2D.hpp>

TEST(Constraints2DTest, ZeroesOutOfPlaneMotion) {
  Vec3f position{5.0f, 7.0f, 3.0f};
  Vec3f linearVelocity{1.0f, 2.0f, 9.0f};
  Vec3f angularVelocity{4.0f, 6.0f, 8.0f};

  apply2DConstraint(position, linearVelocity, angularVelocity);

  EXPECT_FLOAT_EQ(position.x, 5.0f);
  EXPECT_FLOAT_EQ(position.y, 7.0f);
  EXPECT_FLOAT_EQ(position.z, 0.0f);

  EXPECT_FLOAT_EQ(linearVelocity.x, 1.0f);
  EXPECT_FLOAT_EQ(linearVelocity.y, 2.0f);
  EXPECT_FLOAT_EQ(linearVelocity.z, 0.0f);

  EXPECT_FLOAT_EQ(angularVelocity.x, 0.0f);
  EXPECT_FLOAT_EQ(angularVelocity.y, 0.0f);
  EXPECT_FLOAT_EQ(angularVelocity.z, 8.0f);
}
