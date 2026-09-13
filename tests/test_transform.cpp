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

TEST(TransformTest, ComposeAppliesParentThenChild) {
  Transform parent;
  parent.position = Vec3f{1.0f, 0.0f, 0.0f};
  parent.orientation =
      glm::angleAxis(glm::radians(90.0f), Vec3f{0.0f, 0.0f, 1.0f});

  Transform child;
  child.position = Vec3f{2.0f, 0.0f, 0.0f};
  child.orientation =
      glm::angleAxis(glm::radians(90.0f), Vec3f{0.0f, 0.0f, 1.0f});

  Transform composed = compose(parent, child);

  // orientation: two 90-degree Z rotations stack to 180 degrees
  Quatf expectedOrientation =
      glm::angleAxis(glm::radians(180.0f), Vec3f{0.0f, 0.0f, 1.0f});
  EXPECT_NEAR(glm::abs(glm::dot(composed.orientation, expectedOrientation)),
              1.0f, 1e-5f);

  // position: child's local offset (2,0,0) rotated 90 degrees by parent
  // becomes (0,2,0), then added to parent's position (1,0,0)
  EXPECT_NEAR(composed.position.x, 1.0f, 1e-5f);
  EXPECT_NEAR(composed.position.y, 2.0f, 1e-5f);
  EXPECT_NEAR(composed.position.z, 0.0f, 1e-5f);

  // composing with child point SHOULD match manually chaining two transforms
  Vec3f localPoint{0.5f, 0.0f, 0.0f};
  Vec3f viaCompose = transformPoint(composed, localPoint);
  Vec3f viaChaining = transformPoint(parent, transformPoint(child, localPoint));
  EXPECT_NEAR(viaCompose.x, viaChaining.x, 1e-5f);
  EXPECT_NEAR(viaCompose.y, viaChaining.y, 1e-5f);
  EXPECT_NEAR(viaCompose.z, viaChaining.z, 1e-5f);
}
