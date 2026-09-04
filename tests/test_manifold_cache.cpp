#include <gtest/gtest.h>

#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <collision/Manifold.hpp>
#include <collision/ManifoldCache.hpp>
#include <core/BodyHandle.hpp>
#include <math/Transform.hpp>

namespace {
Transform at(float x, float y) {
  Transform t;
  t.position = Vec3f(x, y, 0.0f);
  return t;
}
}  // namespace

TEST(ManifoldCacheTest, GetOrCreateIsOrderIndependent) {
  ManifoldCache cache;
  BodyHandle a{0, 0};
  BodyHandle b{1, 0};

  Manifold& viaAB = cache.getOrCreate(a, b);
  viaAB.pointCount = 1;
  Manifold& viaBA = cache.getOrCreate(b, a);

  EXPECT_EQ(viaBA.pointCount, 1);
  EXPECT_EQ(cache.activeManifolds().size(), 1u);
}

TEST(ManifoldCacheTest, EndFrameDropsUnvisitedPairs) {
  ManifoldCache cache;
  BodyHandle a{0, 0};
  BodyHandle b{1, 0};

  cache.beginFrame();
  cache.getOrCreate(a, b);
  cache.endFrame();
  EXPECT_EQ(cache.activeManifolds().size(), 1u);

  cache.beginFrame();
  cache.endFrame();
  EXPECT_EQ(cache.activeManifolds().size(), 0u);
}

TEST(ManifoldCacheTest, WarmStartAccumulatorCarriesForward) {
  ShapeVariant boxA{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant boxB{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.9f, 0.0f);
  BodyHandle a{0, 0};
  BodyHandle b{1, 0};

  GjkResult gjk = gjkOverlap(boxA, ta, boxB, tb);
  ASSERT_TRUE(gjk.overlapping);
  EpaResult epa = epaPenetration(boxA, ta, boxB, tb, gjk);
  Manifold first = buildManifold(boxA, ta, boxB, tb, a, b, gjk, epa);
  ASSERT_GE(first.pointCount, 1);

  ManifoldCache cache;
  cache.beginFrame();
  cache.updateManifold(first, ta, 0.2f);
  cache.endFrame();

  Manifold& cached = cache.getOrCreate(a, b);
  ASSERT_GE(cached.pointCount, 1);
  cached.points[0].normalImpulse = 42.0f;

  Transform tbNudged = at(1.91f, 0.0f);
  GjkResult gjk2 = gjkOverlap(boxA, ta, boxB, tbNudged);
  ASSERT_TRUE(gjk2.overlapping);
  EpaResult epa2 = epaPenetration(boxA, ta, boxB, tbNudged, gjk2);
  Manifold second = buildManifold(boxA, ta, boxB, tbNudged, a, b, gjk2, epa2);

  cache.beginFrame();
  cache.updateManifold(second, ta, 0.2f);
  cache.endFrame();

  Manifold& updated = cache.getOrCreate(a, b);
  bool anyCarriedForward = false;
  for (int i = 0; i < updated.pointCount; ++i) {
    if (updated.points[static_cast<std::size_t>(i)].normalImpulse == 42.0f) {
      anyCarriedForward = true;
    }
  }
  EXPECT_TRUE(anyCarriedForward);
}
