#include <gtest/gtest.h>
#include <math/Transform.hpp>

TEST(TransformTest, ComposeInverseRoundTrip) {
  Transform t;
  t.position = Vec3f{3.0f, -2.0f, 5.0f};
  // 45 degrees about the axis (1,1,1)/sqrt(3)
  Vec3f axis = glm::normalize(Vec3f{1.0f, 1.0f, 1.0f});
  t.orientation = glm::angleAxis(glm::radians(45.0f), axis);

  Vec3f originalPoint{1.0f, 2.0f, 3.0f};
  Vec3f worldPoint = transformPoint(t, originalPoint);

  Transform inv = inverse(t);
  Vec3f roundTripped = transformPoint(inv, worldPoint);

  EXPECT_NEAR(roundTripped.x, originalPoint.x, 1e-5f);
  EXPECT_NEAR(roundTripped.y, originalPoint.y, 1e-5f);
  EXPECT_NEAR(roundTripped.z, originalPoint.z, 1e-5f);
}
