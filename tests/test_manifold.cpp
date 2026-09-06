#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <collision/Manifold.hpp>
#include <core/BodyHandle.hpp>
#include <math/Transform.hpp>
#include <vector>

namespace {
Transform at(float x, float y) {
  Transform t;
  t.position = Vec3f(x, y, 0.0f);
  return t;
}
}  // namespace

TEST(ManifoldTest, FlushBoxOnBoxProducesTwoPoints2D) {
  ShapeVariant boxA{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant boxB{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.9f, 0.0f);

  GjkResult gjk = gjkOverlap(boxA, ta, boxB, tb);
  ASSERT_TRUE(gjk.overlapping);
  EpaResult epa = epaPenetration(boxA, ta, boxB, tb, gjk);

  BodyHandle a{0, 0};
  BodyHandle b{1, 0};
  Manifold m = buildManifold(boxA, ta, boxB, tb, a, b, gjk, epa);

  ASSERT_EQ(m.pointCount, 2);

  std::vector<float> ys;
  for (int i = 0; i < m.pointCount; ++i) {
    Vec3f worldA = transformPoint(ta, m.points[static_cast<std::size_t>(i)].localAnchorA);
    ys.push_back(worldA.y);
  }
  std::sort(ys.begin(), ys.end());
  EXPECT_NEAR(ys[0], -1.0f, 1e-3f);
  EXPECT_NEAR(ys[1], 1.0f, 1e-3f);
  EXPECT_NEAR(m.points[0].penetration, 0.1f, 1e-3f);
  EXPECT_NEAR(m.points[1].penetration, 0.1f, 1e-3f);
}

TEST(ManifoldTest, FlushBoxOnBoxProducesFourPoints3D) {
  ShapeVariant boxA{BoxShape{Vec3f(1.0f, 1.0f, 1.0f)}};
  ShapeVariant boxB{BoxShape{Vec3f(1.0f, 1.0f, 1.0f)}};
  Transform ta;
  ta.position = Vec3f(0.0f, 0.0f, 0.0f);
  Transform tb;
  tb.position = Vec3f(1.9f, 0.0f, 0.0f);

  GjkResult gjk = gjkOverlap(boxA, ta, boxB, tb);
  ASSERT_TRUE(gjk.overlapping);
  ASSERT_EQ(gjk.simplexCount, 4);
  EpaResult epa = epaPenetration(boxA, ta, boxB, tb, gjk);

  BodyHandle a{0, 0};
  BodyHandle b{1, 0};
  Manifold m = buildManifold(boxA, ta, boxB, tb, a, b, gjk, epa);

  ASSERT_EQ(m.pointCount, 4);
  for (int i = 0; i < m.pointCount; ++i) {
    const ManifoldPoint& p = m.points[static_cast<std::size_t>(i)];
    Vec3f worldA = transformPoint(ta, p.localAnchorA);
    EXPECT_NEAR(std::abs(worldA.y), 1.0f, 1e-3f);
    EXPECT_NEAR(std::abs(worldA.z), 1.0f, 1e-3f);
    EXPECT_NEAR(p.penetration, 0.1f, 1e-3f);
  }
}

TEST(ManifoldTest, ClockwiseWoundHullClipsSameAsCounterclockwise) {
  std::vector<Vec3f> ccw{Vec3f(-1.0f, -1.0f, 0.0f), Vec3f(1.0f, -1.0f, 0.0f),
                        Vec3f(1.0f, 1.0f, 0.0f), Vec3f(-1.0f, 1.0f, 0.0f)};
  std::vector<Vec3f> cw{Vec3f(-1.0f, -1.0f, 0.0f), Vec3f(-1.0f, 1.0f, 0.0f),
                       Vec3f(1.0f, 1.0f, 0.0f), Vec3f(1.0f, -1.0f, 0.0f)};
  ShapeVariant hullA{ConvexHullShape(ccw)};
  ShapeVariant hullB{ConvexHullShape(cw)};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.9f, 0.0f);

  GjkResult gjk = gjkOverlap(hullA, ta, hullB, tb);
  ASSERT_TRUE(gjk.overlapping);
  EpaResult epa = epaPenetration(hullA, ta, hullB, tb, gjk);

  BodyHandle a{0, 0};
  BodyHandle b{1, 0};
  Manifold m = buildManifold(hullA, ta, hullB, tb, a, b, gjk, epa);

  ASSERT_EQ(m.pointCount, 2);
  std::vector<float> ys;
  for (int i = 0; i < m.pointCount; ++i) {
    Vec3f worldA = transformPoint(ta, m.points[static_cast<std::size_t>(i)].localAnchorA);
    ys.push_back(worldA.y);
  }
  std::sort(ys.begin(), ys.end());
  EXPECT_NEAR(ys[0], -1.0f, 1e-3f);
  EXPECT_NEAR(ys[1], 1.0f, 1e-3f);
}

TEST(ManifoldTest, CapsulePairForcedInto2DPathFallsBackToSinglePoint) {
  ShapeVariant capsuleA{CapsuleShape{0.5f, 1.0f}};
  ShapeVariant capsuleB{CapsuleShape{0.5f, 1.0f}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(0.5f, 0.0f);

  GjkResult gjk = gjkOverlap(capsuleA, ta, capsuleB, tb);
  ASSERT_TRUE(gjk.overlapping);
  EpaResult epa = epaPenetration(capsuleA, ta, capsuleB, tb, gjk);

  gjk.simplexCount = 3;
  BodyHandle a{0, 0};
  BodyHandle b{1, 0};
  Manifold m = buildManifold(capsuleA, ta, capsuleB, tb, a, b, gjk, epa);

  EXPECT_EQ(m.pointCount, 1);
}

TEST(ManifoldTest, SpherePairProducesSinglePoint) {
  ShapeVariant a{SphereShape{1.0f}};
  ShapeVariant b{SphereShape{1.0f}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.5f, 0.0f);

  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  ASSERT_TRUE(gjk.overlapping);
  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);

  BodyHandle ha{0, 0};
  BodyHandle hb{1, 0};
  Manifold m = buildManifold(a, ta, b, tb, ha, hb, gjk, epa);

  EXPECT_EQ(m.pointCount, 1);
  EXPECT_NEAR(m.points[0].penetration, 0.5f, 1e-4f);
}
