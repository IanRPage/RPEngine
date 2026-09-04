#include <gtest/gtest.h>

#include <collision/Shapes.hpp>
#include <math/Transform.hpp>

TEST(WorldSupportTest, RotatedBoxSupportMatchesManualTransform) {
  BoxShape box{Vec3f(1.0f, 1.0f, 1.0f)};
  Transform t;
  t.position = Vec3f(5.0f, 0.0f, 0.0f);
  t.orientation = glm::angleAxis(glm::radians(90.0f), Vec3f(0.0f, 0.0f, 1.0f));

  Vec3f worldDir(1.0f, 1.0f, 0.0f);
  Vec3f result = worldSupport(ShapeVariant{box}, t, worldDir);

  EXPECT_NEAR(result.x, 6.0f, 1e-4f);
  EXPECT_NEAR(result.y, 1.0f, 1e-4f);
  EXPECT_NEAR(result.z, 1.0f, 1e-4f);
}

TEST(WorldSupportTest, TranslationDoesNotAffectDirectionRotation) {
  BoxShape box{Vec3f(1.0f, 1.0f, 1.0f)};
  Quatf orientation =
      glm::angleAxis(glm::radians(90.0f), Vec3f(0.0f, 0.0f, 1.0f));
  Vec3f worldDir(1.0f, 0.0f, 0.0f);

  Transform a;
  a.position = Vec3f(0.0f, 0.0f, 0.0f);
  a.orientation = orientation;

  Transform b;
  b.position = Vec3f(10.0f, -3.0f, 7.0f);
  b.orientation = orientation;

  Vec3f resultA = worldSupport(ShapeVariant{box}, a, worldDir);
  Vec3f resultB = worldSupport(ShapeVariant{box}, b, worldDir);
  Vec3f delta = resultB - resultA;

  EXPECT_NEAR(delta.x, b.position.x - a.position.x, 1e-4f);
  EXPECT_NEAR(delta.y, b.position.y - a.position.y, 1e-4f);
  EXPECT_NEAR(delta.z, b.position.z - a.position.z, 1e-4f);
}

TEST(IsFlatPairTest, TwoZeroThicknessBoxesAreFlat) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta, tb;
  tb.position = Vec3f(1.5f, 0.0f, 0.0f);

  EXPECT_TRUE(isFlatPair(a, ta, b, tb));
}

TEST(IsFlatPairTest, RealDepthBoxIsNotFlat) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 1.0f)}};
  Transform ta, tb;
  tb.position = Vec3f(1.5f, 0.0f, 0.0f);

  EXPECT_FALSE(isFlatPair(a, ta, b, tb));
}

TEST(IsFlatPairTest, CapsuleIsNeverFlat) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant b{CapsuleShape{0.5f, 1.0f}};
  Transform ta, tb;
  tb.position = Vec3f(1.5f, 0.0f, 0.0f);

  EXPECT_FALSE(isFlatPair(a, ta, b, tb));
}
