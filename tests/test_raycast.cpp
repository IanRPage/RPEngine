#include <gtest/gtest.h>

#include <collision/Raycast.hpp>
#include <core/World.hpp>
#include <math/Constants.hpp>
#include <random>

namespace {

constexpr float kTol = 1e-4f;

void expectVecNear(Vec3f actual, Vec3f expected, float tol = kTol) {
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

Transform at(Vec3f p, Quatf q = Quatf(1.0f, 0.0f, 0.0f, 0.0f)) {
  return Transform{p, q};
}

}  // namespace

TEST(RaycastShapeTest, SphereHitAndMiss) {
  ShapeVariant s{SphereShape{1.0f}};
  Transform t = at(Vec3f(5.0f, 0.0f, 0.0f));
  RayHit hit =
      raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(1.0f, 0.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 4.0f, kTol);
  expectVecNear(hit.point, Vec3f(4.0f, 0.0f, 0.0f));
  expectVecNear(hit.normal, Vec3f(-1.0f, 0.0f, 0.0f));

  RayHit nonUnit =
      raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(nonUnit.hit);
  EXPECT_NEAR(nonUnit.distance, 4.0f, kTol);

  EXPECT_FALSE(
      raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(1.0f, 0.0f, 0.0f)}, 3.9f).hit);
  EXPECT_TRUE(
      raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(1.0f, 0.0f, 0.0f)}, 4.1f).hit);
  EXPECT_FALSE(
      raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(-1.0f, 0.0f, 0.0f)}, 100.0f)
          .hit);
  EXPECT_FALSE(
      raycastShape(s, t, Ray{Vec3f(0.0f, 2.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)},
                   100.0f)
          .hit);
  EXPECT_FALSE(raycastShape(s, t, Ray{Vec3f(0.0f), Vec3f(0.0f)}, 100.0f).hit);
}

TEST(RaycastShapeTest, StartInsideReportsExitWithOutwardNormal) {
  ShapeVariant sphere{SphereShape{2.0f}};
  RayHit hit = raycastShape(sphere, at(Vec3f(0.0f)),
                            Ray{Vec3f(0.0f), Vec3f(0.0f, 1.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 2.0f, kTol);
  expectVecNear(hit.point, Vec3f(0.0f, 2.0f, 0.0f));
  expectVecNear(hit.normal, Vec3f(0.0f, 1.0f, 0.0f));

  ShapeVariant box{BoxShape{Vec3f(1.0f, 2.0f, 3.0f)}};
  hit = raycastShape(box, at(Vec3f(0.0f)),
                     Ray{Vec3f(0.0f), Vec3f(0.0f, 0.0f, -1.0f)}, 100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 3.0f, kTol);
  expectVecNear(hit.normal, Vec3f(0.0f, 0.0f, -1.0f));

  ShapeVariant capsule{CapsuleShape{0.5f, 1.0f}};
  hit = raycastShape(capsule, at(Vec3f(0.0f)),
                     Ray{Vec3f(0.0f), Vec3f(0.0f, 1.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 1.5f, kTol);
  expectVecNear(hit.normal, Vec3f(0.0f, 1.0f, 0.0f));
}

TEST(RaycastShapeTest, RotatedBox) {
  ShapeVariant s{BoxShape{Vec3f(1.0f, 0.5f, 0.5f)}};
  Quatf q = glm::angleAxis(PI * 0.5f, Vec3f(0.0f, 0.0f, 1.0f));
  RayHit hit =
      raycastShape(s, at(Vec3f(0.0f, 0.0f, 0.0f), q),
                   Ray{Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, -1.0f, 0.0f)},
                   100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 9.0f, kTol);
  expectVecNear(hit.normal, Vec3f(0.0f, 1.0f, 0.0f));

  hit = raycastShape(s, at(Vec3f(0.0f), q),
                     Ray{Vec3f(10.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)},
                     100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 9.5f, kTol);
  expectVecNear(hit.normal, Vec3f(1.0f, 0.0f, 0.0f));
}

TEST(RaycastShapeTest, CapsuleSideAndCap) {
  ShapeVariant s{CapsuleShape{0.5f, 1.0f}};
  Transform t = at(Vec3f(0.0f));
  RayHit side = raycastShape(
      s, t, Ray{Vec3f(5.0f, 0.5f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(side.hit);
  EXPECT_NEAR(side.distance, 4.5f, kTol);
  expectVecNear(side.normal, Vec3f(1.0f, 0.0f, 0.0f));

  RayHit cap = raycastShape(
      s, t, Ray{Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f, 1.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(cap.hit);
  EXPECT_NEAR(cap.distance, 3.5f, kTol);
  expectVecNear(cap.normal, Vec3f(0.0f, -1.0f, 0.0f));

  Quatf q = glm::angleAxis(PI * 0.5f, Vec3f(0.0f, 0.0f, 1.0f));
  RayHit rotated = raycastShape(
      s, at(Vec3f(0.0f), q),
      Ray{Vec3f(5.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(rotated.hit);
  EXPECT_NEAR(rotated.distance, 3.5f, kTol);
  expectVecNear(rotated.normal, Vec3f(1.0f, 0.0f, 0.0f));
}

TEST(RaycastShapeTest, FlatHullHitFromFrontAndInPlane) {
  ShapeVariant s{ConvexHullShape({Vec3f(-1.0f, -1.0f, 0.0f),
                                  Vec3f(1.0f, -1.0f, 0.0f),
                                  Vec3f(1.0f, 1.0f, 0.0f),
                                  Vec3f(-1.0f, 1.0f, 0.0f)})};
  Transform t = at(Vec3f(2.0f, 0.0f, 0.0f));

  RayHit front = raycastShape(
      s, t, Ray{Vec3f(2.5f, 0.5f, 10.0f), Vec3f(0.0f, 0.0f, -1.0f)}, 100.0f);
  ASSERT_TRUE(front.hit);
  EXPECT_NEAR(front.distance, 10.0f, kTol);
  expectVecNear(front.normal, Vec3f(0.0f, 0.0f, 1.0f));

  RayHit back = raycastShape(
      s, t, Ray{Vec3f(2.5f, 0.5f, -10.0f), Vec3f(0.0f, 0.0f, 1.0f)}, 100.0f);
  ASSERT_TRUE(back.hit);
  expectVecNear(back.normal, Vec3f(0.0f, 0.0f, -1.0f));

  RayHit inPlane = raycastShape(
      s, t, Ray{Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(inPlane.hit);
  EXPECT_NEAR(inPlane.distance, 6.0f, kTol);
  expectVecNear(inPlane.normal, Vec3f(-1.0f, 0.0f, 0.0f));

  EXPECT_FALSE(raycastShape(s, t,
                            Ray{Vec3f(5.0f, 0.5f, 10.0f),
                                Vec3f(0.0f, 0.0f, -1.0f)},
                            100.0f)
                   .hit);
}

TEST(RaycastShapeTest, SolidHullMatchesBox) {
  std::vector<Vec3f> corners;
  for (float x : {-1.0f, 1.0f}) {
    for (float y : {-2.0f, 2.0f}) {
      for (float z : {-0.5f, 0.5f}) { corners.emplace_back(x, y, z); }
    }
  }
  ShapeVariant hull{ConvexHullShape(corners)};
  ShapeVariant box{BoxShape{Vec3f(1.0f, 2.0f, 0.5f)}};
  Quatf q = glm::normalize(Quatf(0.9f, 0.2f, -0.3f, 0.1f));
  Transform t = at(Vec3f(1.0f, 2.0f, 3.0f), q);

  std::mt19937 rng(3);
  std::uniform_real_distribution<float> dist(-6.0f, 6.0f);
  int hits = 0;
  for (int i = 0; i < 500; ++i) {
    Ray ray{Vec3f(dist(rng), dist(rng), dist(rng)) + t.position,
            Vec3f(dist(rng), dist(rng), dist(rng))};
    RayHit a = raycastShape(hull, t, ray, 50.0f);
    RayHit b = raycastShape(box, t, ray, 50.0f);
    ASSERT_EQ(a.hit, b.hit);
    if (!a.hit) { continue; }
    ++hits;
    EXPECT_NEAR(a.distance, b.distance, 1e-3f);
    expectVecNear(a.point, b.point, 1e-3f);
  }
  EXPECT_GT(hits, 20);
}

TEST(RaycastShapeTest, CompoundShapePicksClosestChild) {
  CompoundShape compound({
      CompoundChild{SphereShape{1.0f}, at(Vec3f(-2.0f, 0.0f, 0.0f)), 1.0f},
      CompoundChild{BoxShape{Vec3f(1.0f)}, at(Vec3f(2.0f, 0.0f, 0.0f)), 1.0f},
  });
  ShapeVariant s{compound};
  Transform t =
      at(Vec3f(0.0f, 3.0f, 0.0f),
         glm::angleAxis(PI * 0.5f, Vec3f(0.0f, 0.0f, 1.0f)));

  RayHit hit = raycastShape(
      s, t, Ray{Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, -1.0f, 0.0f)}, 100.0f);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 4.0f, kTol);
  expectVecNear(hit.point, Vec3f(0.0f, 6.0f, 0.0f));
  expectVecNear(hit.normal, Vec3f(0.0f, 1.0f, 0.0f));

  EXPECT_FALSE(
      raycastShape(s, t,
                   Ray{Vec3f(0.0f, 3.0f, 10.0f), Vec3f(0.0f, 0.0f, -1.0f)},
                   100.0f)
          .hit);
}

namespace {

void populateWorld(World& world) {
  world.createStaticBody(BoxShape{Vec3f(20.0f, 0.5f, 20.0f)},
                         at(Vec3f(0.0f, -0.5f, 0.0f)), 0.5f, 0.0f);
  world.createDynamicBody(SphereShape{0.5f}, at(Vec3f(0.0f, 2.0f, 0.0f)), 1.0f,
                          0.5f, 0.0f);
  world.createDynamicBody(BoxShape{Vec3f(0.5f)}, at(Vec3f(0.0f, 5.0f, 0.0f)),
                          1.0f, 0.5f, 0.0f);
  world.createDynamicBody(CapsuleShape{0.3f, 0.5f},
                          at(Vec3f(4.0f, 2.0f, 0.0f)), 1.0f, 0.5f, 0.0f);
}

}  // namespace

TEST(WorldQueryTest, RaycastClosestAndAll) {
  World world;
  populateWorld(world);
  Ray down{Vec3f(0.0f, 20.0f, 0.0f), Vec3f(0.0f, -1.0f, 0.0f)};

  auto closest = world.raycastClosest(down);
  ASSERT_TRUE(closest.has_value());
  EXPECT_EQ(closest->body.index, 2u);
  EXPECT_NEAR(closest->distance, 14.5f, kTol);
  expectVecNear(closest->normal, Vec3f(0.0f, 1.0f, 0.0f));

  auto all = world.raycastAll(down);
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0].body.index, 2u);
  EXPECT_EQ(all[1].body.index, 1u);
  EXPECT_EQ(all[2].body.index, 0u);
  EXPECT_NEAR(all[1].distance, 17.5f, kTol);
  EXPECT_NEAR(all[2].distance, 20.0f, kTol);

  EXPECT_EQ(world.raycastAll(down, 16.0f).size(), 1u);
  EXPECT_FALSE(world.raycastClosest(down, 10.0f).has_value());
}

TEST(WorldQueryTest, ReflectsMovesRotationsAndRemovals) {
  World world;
  populateWorld(world);
  world.step(1.0f / 60.0f);

  BodyHandle box{2, 0};
  Ray side{Vec3f(-10.0f, 5.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)};
  auto hit = world.raycastClosest(side);
  ASSERT_TRUE(hit);
  EXPECT_EQ(hit->body, box);
  float before = hit->distance;

  world.bodies().orientation(box) =
      glm::angleAxis(PI * 0.25f, Vec3f(0.0f, 0.0f, 1.0f));
  hit = world.raycastClosest(side);
  ASSERT_TRUE(hit);
  EXPECT_LT(hit->distance, before - 0.1f);

  world.bodies().position(box) = Vec3f(30.0f, 30.0f, 0.0f);
  EXPECT_FALSE(world.raycastClosest(side).has_value());
  auto found = world.queryAABB(AABB(Vec3f(29.0f, 29.0f, -1.0f), Vec3f(31.0f, 31.0f, 1.0f)));
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0], box);

  world.removeBody(box);
  EXPECT_TRUE(world.queryAABB(AABB(Vec3f(29.0f, 29.0f, -1.0f), Vec3f(31.0f, 31.0f, 1.0f))).empty());

  BodyHandle added = world.createDynamicBody(
      SphereShape{1.0f}, at(Vec3f(0.0f, 5.0f, 0.0f)), 1.0f, 0.5f, 0.0f);
  hit = world.raycastClosest(side);
  ASSERT_TRUE(hit);
  EXPECT_EQ(hit->body, added);
  EXPECT_NEAR(hit->distance, 9.0f, kTol);
}

TEST(WorldQueryTest, QueryAABB) {
  World world;
  populateWorld(world);
  auto found = world.queryAABB(AABB(Vec3f(-1.0f, 1.0f, -1.0f),
                                    Vec3f(1.0f, 3.0f, 1.0f)));
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].index, 1u);
  EXPECT_EQ(world.queryAABB(AABB(Vec3f(-50.0f), Vec3f(50.0f))).size(), 4u);
}
