#include <gtest/gtest.h>

#include <cmath>
#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <math/Transform.hpp>

namespace {
Transform at(float x, float y, float z = 0.0f) {
  Transform t;
  t.position = Vec3f(x, y, z);
  return t;
}
}  // namespace

TEST(EpaTest, SphereVsSpherePenetrationMatchesAnalytic) {
  ShapeVariant a{SphereShape{2.0f}};
  ShapeVariant b{SphereShape{3.0f}};
  Transform ta = at(0.0f, 0.0f, 0.0f);
  Transform tb = at(4.0f, 0.0f, 0.0f);

  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  ASSERT_TRUE(gjk.overlapping);

  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);
  float expectedDepth = 2.0f + 3.0f - 4.0f;
  EXPECT_NEAR(epa.penetrationDepth, expectedDepth, 1e-4f);
  EXPECT_NEAR(epa.normal.x, 1.0f, 1e-4f);
  EXPECT_NEAR(epa.normal.y, 0.0f, 1e-4f);
  EXPECT_NEAR(epa.normal.z, 0.0f, 1e-4f);
}

TEST(EpaTest, SphereVsSphereNormalPointsFromAToB) {
  ShapeVariant a{SphereShape{1.0f}};
  ShapeVariant b{SphereShape{1.0f}};
  Transform ta = at(0.0f, 0.0f, 0.0f);
  Transform tb = at(0.0f, 1.0f, 0.0f);

  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  ASSERT_TRUE(gjk.overlapping);

  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);
  EXPECT_NEAR(epa.normal.x, 0.0f, 1e-4f);
  EXPECT_NEAR(epa.normal.y, 1.0f, 1e-4f);
}

TEST(EpaTest, BoxVsBoxPenetration2D) {
  ShapeVariant a{BoxShape{Vec3f(5.0f, 5.0f, 0.0f)}};
  ShapeVariant b{BoxShape{Vec3f(5.0f, 5.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(9.0f, 0.0f);

  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  ASSERT_TRUE(gjk.overlapping);

  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);
  EXPECT_NEAR(epa.penetrationDepth, 1.0f, 1e-3f);
  EXPECT_NEAR(std::abs(epa.normal.x), 1.0f, 1e-3f);
}

TEST(EpaTest, CircleVsBoxPenetration2D) {
  ShapeVariant circle{SphereShape{10.0f}};
  ShapeVariant rect{BoxShape{Vec3f(5.0f, 5.0f, 20.0f)}};
  Transform tc = at(0.0f, 0.0f);
  Transform tr = at(5.0f, 0.0f);  // x in [0, 10], y in [-5, 5]

  GjkResult gjk = gjkOverlap(circle, tc, rect, tr);
  ASSERT_TRUE(gjk.overlapping);

  EpaResult epa = epaPenetration(circle, tc, rect, tr, gjk);
  EXPECT_NEAR(epa.penetrationDepth, 10.0f, 1e-3f);
  EXPECT_NEAR(epa.normal.x, 1.0f, 1e-3f);
}

TEST(EpaTest, BoxVsBoxPenetration3D) {
  ShapeVariant a{BoxShape{Vec3f(5.0f, 5.0f, 5.0f)}};
  ShapeVariant b{BoxShape{Vec3f(5.0f, 5.0f, 5.0f)}};
  Transform ta = at(0.0f, 0.0f, 0.0f);
  Transform tb = at(9.0f, 0.0f, 0.0f);

  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  ASSERT_TRUE(gjk.overlapping);
  ASSERT_EQ(gjk.simplexCount, 4);

  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);
  EXPECT_NEAR(epa.penetrationDepth, 1.0f, 1e-3f);
  EXPECT_NEAR(std::abs(epa.normal.x), 1.0f, 1e-3f);
}
