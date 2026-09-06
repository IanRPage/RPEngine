#include <gtest/gtest.h>

#include <broadphase/DynamicBVHBroadphase.hpp>
#include <cmath>
#include <core/World.hpp>
#include <memory>

namespace {

class CountingBroadphase : public IBroadphase {
 public:
  explicit CountingBroadphase(std::unique_ptr<IBroadphase> inner)
      : inner_(std::move(inner)) {}

  std::span<const std::pair<BodyHandle, BodyHandle>> computePairs(
      const BodyStore& bodies) override {
    callCount++;
    return inner_->computePairs(bodies);
  }

  int callCount = 0;

 private:
  std::unique_ptr<IBroadphase> inner_;
};

}  // namespace

TEST(SolverLoopStructureTest, BroadphaseAndNarrowphaseCalledExactlyOnceCePerStep) {
  World world;

  auto counting = std::make_unique<CountingBroadphase>(
      std::make_unique<DynamicBVHBroadphase>());
  CountingBroadphase* countingPtr = counting.get();
  world.setBroadphase(std::move(counting));

  int gjkCalls = 0;
  int epaCalls = 0;
  world.setNarrowphaseFns(
      [&](const ShapeVariant& a, const Transform& ta, const ShapeVariant& b,
         const Transform& tb) {
        gjkCalls++;
        return gjkOverlap(a, ta, b, tb);
      },
      [&](const ShapeVariant& a, const Transform& ta, const ShapeVariant& b,
         const Transform& tb, const GjkResult& gjk) {
        epaCalls++;
        return epaPenetration(a, ta, b, tb, gjk);
      });

  Transform ta;
  ta.position = Vec3f(0.0f, 0.0f, 0.0f);
  Transform tb;
  tb.position = Vec3f(0.5f, 0.0f, 0.0f);
  world.createDynamicBody(ShapeVariant{SphereShape{1.0f}}, ta, 1.0f, 0.5f,
                          0.0f, true);
  world.createDynamicBody(ShapeVariant{SphereShape{1.0f}}, tb, 1.0f, 0.5f,
                          0.0f, true);

  world.config().velocityIterations = 8;
  world.config().positionIterations = 4;

  world.step(1.0f / 60.0f);

  EXPECT_EQ(countingPtr->callCount, 1);
  EXPECT_EQ(gjkCalls, 1);
  EXPECT_EQ(epaCalls, 1);
}

TEST(WorldTest, WorldBoundariesConstrainFallingBody) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));
  world.addWorldBoundaries(Vec3f(-10.0f, -10.0f, 0.0f), Vec3f(10.0f, 10.0f, 0.0f),
                          1.0f, /*friction=*/0.5f, /*restitution=*/0.3f,
                          /*is2D=*/true);

  Transform sphereT;
  sphereT.position = Vec3f(0.0f, 5.0f, 0.0f);
  BodyHandle sphere = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, sphereT, 1.0f, 0.5f, 0.3f, true);

  const float dt = 1.0f / 120.0f;
  for (int i = 0; i < 600; i++) {
    world.step(dt);
    Vec3f pos = world.bodies().position(sphere);
    ASSERT_FALSE(std::isnan(pos.y));
    ASSERT_GE(pos.y, -10.0f - 0.5f - 0.5f);
    ASSERT_LE(pos.y, 10.0f + 0.5f + 0.5f);
  }
}
