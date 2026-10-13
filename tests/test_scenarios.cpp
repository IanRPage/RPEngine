#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <core/World.hpp>

#include "../benchmarks/scenes.hpp"
#include "test_helpers.hpp"

using test_helpers::totalKineticEnergy;

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr int kSteps = 600;
constexpr int kCheckEvery = 60;

World makeWorld() {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));
  return world;
}

std::string at(int step) {
  return "t=" + std::to_string(static_cast<float>(step) * kDt) + "s";
}

// drops a sphere from h0 and returns height of first rebound apex above the
// resting height (0 if it never bounced)
float sphereBounceHeight(float restitution, float h0) {
  constexpr float kRadius = 0.5f;
  World world = makeWorld();
  scenes::addGround(world, /*friction=*/0.0f);
  Transform t;
  t.position = Vec3f(0.0f, kRadius + h0, 0.0f);
  BodyHandle sphere = world.createDynamicBody(
      ShapeVariant{SphereShape{kRadius}}, t, /*mass=*/1.0f, /*friction=*/0.0f,
      restitution);

  bool wasFalling = false;
  bool bounced = false;
  float apexY = kRadius;
  for (int i = 0; i < kSteps; i++) {
    world.step(kDt);
    const float y = world.bodies().position(sphere).y;
    const float vy = world.bodies().linearVelocity(sphere).y;
    if (!std::isfinite(y) || !std::isfinite(vy)) { return -1.0f; }
    if (!bounced) {
      if (wasFalling && vy > 0.0f) { bounced = true; }
      wasFalling = vy < 0.0f;
    } else {
      if (vy <= 0.0f) { break; }
      apexY = std::max(apexY, y);
    }
  }
  return apexY - kRadius;
}

// stack of unit boxes with 1 cm gap. after 5s every box must be at rest
void expectBoxStackSettles(int height) {
  constexpr float kHalf = 0.5f;
  constexpr int kSettleSteps = 300;
  World world = makeWorld();
  const std::vector<BodyHandle> boxes =
      scenes::buildBoxStack(world, height).bodies;
  const BodyStore& bodies = world.bodies();
  const Vec3f topStart = bodies.position(boxes.back());
  const float idealTopY = 2.0f * kHalf * static_cast<float>(height) - kHalf;
  const float sinkTolerance =
      static_cast<float>(height + 1) * 2.0f * world.config().slop;

  for (int step = 1; step <= kSteps; step++) {
    world.step(kDt);
    if (step < kSettleSteps || step % kCheckEvery != 0) { continue; }
    SCOPED_TRACE(at(step));
    // NaNs fail these comparisons too
    for (size_t i = 0; i < boxes.size(); i++) {
      ASSERT_LT(glm::length(bodies.linearVelocity(boxes[i])), 0.05f)
          << "box " << i << " still moving";
      ASSERT_LT(glm::length(bodies.angularVelocity(boxes[i])), 0.1f)
          << "box " << i << " still rotating";
    }
    const Vec3f top = bodies.position(boxes.back());
    ASSERT_LT(std::hypot(top.x - topStart.x, top.z - topStart.z), 0.05f)
        << "top box drifted sideways";
    ASSERT_NEAR(top.y, idealTopY, sinkTolerance) << "stack sank or floated";
  }
}

// nothing goes NaN, escapes the walls, or blows up. pile settles by 10s
void expectPileStaysContained(bool mixed, int n) {
  World world = makeWorld();
  const scenes::Scene pile = scenes::buildPile(world, mixed, n);
  const BodyStore& bodies = world.bodies();

  for (int step = 1; step <= kSteps; step++) {
    world.step(kDt);
    if (step % kCheckEvery != 0) { continue; }
    SCOPED_TRACE(at(step));
    for (size_t i = 0; i < pile.bodies.size(); i++) {
      const Vec3f p = bodies.position(pile.bodies[i]);
      ASSERT_TRUE(pile.contains(p))
          << "body " << i << " escaped the walls or went NaN at (" << p.x
          << ", " << p.y << ", " << p.z << ")";
      ASSERT_LT(glm::length(bodies.linearVelocity(pile.bodies[i])), 50.0f)
          << "body " << i << " blew up";
    }
  }
  const float meanKe =
      totalKineticEnergy(bodies, pile.bodies) / static_cast<float>(n);
  EXPECT_LT(meanKe, 0.05f) << "pile never settled (mean KE per body, J)";
}

}  // namespace

TEST(ScenarioTest, SphereDropBounceMatchesRestitution) {
  const float h0 = 5.0f;
  for (float e : {0.0f, 0.5f, 0.8f}) {
    SCOPED_TRACE("restitution=" + std::to_string(e));
    const float expected = e * e * h0;
    const float h1 = sphereBounceHeight(e, h0);
    ASSERT_GE(h1, 0.0f) << "sphere state became NaN";
    EXPECT_NEAR(h1, expected, 0.3f * expected + 0.1f);
  }
}

TEST(ScenarioTest, BoxStackOf5Settles) { expectBoxStackSettles(5); }

TEST(ScenarioTest, DISABLED_BoxStackOf10Settles) { expectBoxStackSettles(10); }
TEST(ScenarioTest, DISABLED_BoxStackOf20Settles) { expectBoxStackSettles(20); }

TEST(ScenarioTest, SpherePileStaysInsideWorldBoundaries) {
  expectPileStaysContained(/*mixed=*/false, 216);
}

TEST(ScenarioTest, MixedShapesPileStaysStable) {
  expectPileStaysContained(/*mixed=*/true, 150);
}
