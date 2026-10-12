#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <core/World.hpp>

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

void addGround(World& world, float friction) {
  Transform t;
  t.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(50.0f, 0.5f, 50.0f)}}, t,
                         friction, /*restitution=*/0.0f);
}

bool isFinite(Vec3f v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::string at(int step) {
  return "t=" + std::to_string(static_cast<float>(step) * kDt) + "s";
}

// drops a sphere from h0 and returns height of first rebound apex above the
// resting height (0 if it never bounced)
float sphereBounceHeight(float restitution, float h0) {
  constexpr float kRadius = 0.5f;
  World world = makeWorld();
  addGround(world, /*friction=*/0.0f);
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
  addGround(world, /*friction=*/0.5f);
  std::vector<BodyHandle> boxes;
  for (int i = 0; i < height; i++) {
    Transform t;
    t.position = Vec3f(0.0f, kHalf + static_cast<float>(i) * 1.01f, 0.0f);
    boxes.push_back(world.createDynamicBody(
        ShapeVariant{BoxShape{Vec3f(kHalf)}}, t, /*mass=*/1.0f,
        /*friction=*/0.5f, /*restitution=*/0.0f));
  }
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

// same as buildPile in benchmarks/bench_physics.cpp. `n` bodies on a jittered,
// layered grid inside addWorldBoundaries walls
struct Pile {
  std::vector<BodyHandle> bodies;
  Vec3f min{0.0f};
  Vec3f max{0.0f};
};

Pile buildPile(World& world, bool mixed, int n) {
  const float cell = mixed ? 1.5f : 1.2f;
  const int layers =
      std::max(1, static_cast<int>(std::lround(std::cbrt(n) / 2.0)));
  const int k = static_cast<int>(std::ceil(std::sqrt(double(n) / layers)));
  const int usedLayers = (n + k * k - 1) / (k * k);
  const float halfWidth = 0.5f * (static_cast<float>(k) * cell + 0.5f);
  const float height = static_cast<float>(usedLayers) * cell + 1.5f;

  Pile pile;
  pile.min = Vec3f(-halfWidth, 0.0f, -halfWidth);
  pile.max = Vec3f(halfWidth, height, halfWidth);
  world.addWorldBoundaries(pile.min, pile.max, /*thickness=*/1.0f,
                           /*friction=*/0.5f, /*restitution=*/0.2f,
                           /*is2D=*/false);

  std::mt19937 rng(0x5EED);
  std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
  std::normal_distribution<float> gauss;
  for (int i = 0; i < n; i++) {
    const int layer = i / (k * k);
    const int ix = (i % (k * k)) % k;
    const int iz = (i % (k * k)) / k;
    Transform t;
    t.position = Vec3f(
        -halfWidth + 0.25f + cell * (static_cast<float>(ix) + 0.5f) + jitter(rng),
        0.25f + cell * (static_cast<float>(layer) + 0.5f),
        -halfWidth + 0.25f + cell * (static_cast<float>(iz) + 0.5f) + jitter(rng));

    ShapeVariant shape{SphereShape{0.5f}};
    if (mixed) {
      t.orientation = glm::normalize(
          Quatf(gauss(rng), gauss(rng), gauss(rng), gauss(rng)));
      if (i % 3 == 0) { shape = ShapeVariant{BoxShape{Vec3f(0.4f)}}; }
      if (i % 3 == 2) { shape = ShapeVariant{CapsuleShape{0.3f, 0.4f}}; }
    }
    pile.bodies.push_back(world.createDynamicBody(
        shape, t, /*mass=*/1.0f, /*friction=*/0.5f, /*restitution=*/0.2f));
  }
  return pile;
}

// nothing goes NaN, escapes the walls, or blows up. pile settles by 10s
void expectPileStaysContained(bool mixed, int n) {
  World world = makeWorld();
  const Pile pile = buildPile(world, mixed, n);
  const BodyStore& bodies = world.bodies();

  for (int step = 1; step <= kSteps; step++) {
    world.step(kDt);
    if (step % kCheckEvery != 0) { continue; }
    SCOPED_TRACE(at(step));
    for (size_t i = 0; i < pile.bodies.size(); i++) {
      const Vec3f p = bodies.position(pile.bodies[i]);
      ASSERT_TRUE(isFinite(p)) << "body " << i << " became NaN";
      ASSERT_TRUE(p.x >= pile.min.x && p.x <= pile.max.x &&
                  p.y >= pile.min.y && p.y <= pile.max.y &&
                  p.z >= pile.min.z && p.z <= pile.max.z)
          << "body " << i << " escaped the walls at (" << p.x << ", " << p.y
          << ", " << p.z << ")";
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
