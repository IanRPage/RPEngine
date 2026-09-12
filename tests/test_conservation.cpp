#include <gtest/gtest.h>

#include <array>

#include <core/World.hpp>

#include "test_helpers.hpp"

using test_helpers::stepWorld;
using test_helpers::systemCenterOfMass;
using test_helpers::totalAngularMomentumAboutPoint;
using test_helpers::totalLinearMomentum;

namespace {
constexpr float kDt = 1.0f / 240.0f;
}  // namespace

TEST(ConservationTest, MultiBodyMomentumConservedNoExternalForce) {
  World world;
  world.setGravity(Vec3f(0.0f));

  std::array<BodyHandle, 4> bodies{
      world.createDynamicBody(ShapeVariant{SphereShape{0.5f}},
                              Transform{Vec3f(-6.0f, 0.1f, 0.0f)},
                              /*mass=*/1.0f, /*friction=*/0.0f,
                              /*restitution=*/1.0f),
      world.createDynamicBody(ShapeVariant{BoxShape{Vec3f(0.5f)}},
                              Transform{Vec3f(6.0f, -0.1f, 0.0f)},
                              /*mass=*/2.0f, /*friction=*/0.0f,
                              /*restitution=*/1.0f),
      world.createDynamicBody(ShapeVariant{CapsuleShape{0.3f, 0.3f}},
                              Transform{Vec3f(0.0f, -6.0f, 0.2f)},
                              /*mass=*/1.5f, /*friction=*/0.0f,
                              /*restitution=*/1.0f),
      world.createDynamicBody(ShapeVariant{SphereShape{0.4f}},
                              Transform{Vec3f(0.2f, 6.0f, -0.2f)},
                              /*mass=*/0.8f, /*friction=*/0.0f,
                              /*restitution=*/1.0f),
  };
  world.bodies().linearVelocity(bodies[0]) = Vec3f(3.0f, 0.0f, 0.0f);
  world.bodies().linearVelocity(bodies[1]) = Vec3f(-3.0f, 0.0f, 0.0f);
  world.bodies().linearVelocity(bodies[2]) = Vec3f(0.0f, 3.0f, 0.0f);
  world.bodies().linearVelocity(bodies[3]) = Vec3f(0.0f, -3.0f, 0.0f);

  Vec3f momentumBefore = totalLinearMomentum(world.bodies(), bodies);
  std::array<Vec3f, 4> velocityBefore;
  for (int i = 0; i < 4; i++) {
    velocityBefore[i] = world.bodies().linearVelocity(bodies[i]);
  }

  stepWorld(world, 700, kDt);  // ~2.9s enough for all 4 to converge & pass

  bool anyVelocityChanged = false;
  for (int i = 0; i < 4; i++) {
    if (glm::length(world.bodies().linearVelocity(bodies[i]) -
                    velocityBefore[i]) > 0.1f) {
      anyVelocityChanged = true;
    }
  }
  ASSERT_TRUE(anyVelocityChanged)
      << "no body's velocity changed -- scenario didn't actually collide";

  Vec3f momentumAfter = totalLinearMomentum(world.bodies(), bodies);
  EXPECT_NEAR(glm::length(momentumAfter - momentumBefore), 0.0f, 1e-3f);
}

TEST(ConservationTest, AngularMomentumConservedForIsolatedSystem) {
  World world;
  world.setGravity(Vec3f(0.0f));

  BodyHandle a = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(-3.0f, 0.3f, 0.0f)},
      /*mass=*/1.0f, /*friction=*/0.8f, /*restitution=*/0.5f);
  BodyHandle b = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(3.0f, -0.3f, 0.0f)},
      /*mass=*/1.0f, /*friction=*/0.8f, /*restitution=*/0.5f);
  world.bodies().linearVelocity(a) = Vec3f(2.0f, 0.0f, 0.0f);
  world.bodies().linearVelocity(b) = Vec3f(-2.0f, 0.0f, 0.0f);

  std::array<BodyHandle, 2> bodies{a, b};
  Vec3f referencePoint = systemCenterOfMass(world.bodies(), bodies);

  Vec3f before =
      totalAngularMomentumAboutPoint(world.bodies(), bodies, referencePoint);

  stepWorld(world, 450, kDt);

  ASSERT_GT(glm::length(world.bodies().angularVelocity(a)), 1e-3f)
      << "body A never rotated -- scenario didn't actually exercise spin";

  Vec3f after =
      totalAngularMomentumAboutPoint(world.bodies(), bodies, referencePoint);
  EXPECT_NEAR(glm::length(after - before), 0.0f, 1e-3f);
}

TEST(ConservationTest, CenterOfMassMovesLinearlyUnderGravityAloneContactsOnly) {
  World world;
  world.setGravity(Vec3f(0.0f));

  BodyHandle a = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(-2.0f, 0.0f, 0.0f)},
      /*mass=*/1.0f, /*friction=*/0.0f, /*restitution=*/0.6f);
  BodyHandle b = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(2.0f, 0.0f, 0.0f)},
      /*mass=*/1.0f, /*friction=*/0.0f, /*restitution=*/0.6f);
  world.bodies().linearVelocity(a) = Vec3f(2.0f, 0.0f, 0.0f);
  world.bodies().linearVelocity(b) = Vec3f(-2.0f, 0.0f, 0.0f);

  std::array<BodyHandle, 2> bodies{a, b};
  Vec3f comStart = systemCenterOfMass(world.bodies(), bodies);
  ASSERT_NEAR(glm::length(comStart), 0.0f, 1e-5f);

  stepWorld(world, 300, kDt);

  Vec3f comEnd = systemCenterOfMass(world.bodies(), bodies);
  EXPECT_NEAR(glm::length(comEnd - comStart), 0.0f, 1e-5f);
}

TEST(ConservationTest, CenterOfMassMovesLinearlyUnderGravityAloneGravityOnly) {
  World world;
  const Vec3f gravity(0.0f, -9.81f, 0.0f);
  world.setGravity(gravity);

  BodyHandle a = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(-20.0f, 0.0f, 0.0f)},
      /*mass=*/1.0f, /*friction=*/0.0f, /*restitution=*/0.0f);
  BodyHandle b = world.createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, Transform{Vec3f(20.0f, 0.0f, 0.0f)},
      /*mass=*/2.0f, /*friction=*/0.0f, /*restitution=*/0.0f);
  Vec3f v0a(1.0f, 3.0f, 0.0f);
  Vec3f v0b(-0.5f, 1.0f, 0.0f);
  world.bodies().linearVelocity(a) = v0a;
  world.bodies().linearVelocity(b) = v0b;

  std::array<BodyHandle, 2> bodies{a, b};
  Vec3f com0 = systemCenterOfMass(world.bodies(), bodies);
  float massA = 1.0f, massB = 2.0f;
  Vec3f v0Com = (massA * v0a + massB * v0b) / (massA + massB);

  const int steps = 240;
  stepWorld(world, steps, kDt);
  float t = steps * kDt;

  float n = static_cast<float>(steps);
  Vec3f expected =
      com0 + v0Com * t + gravity * (kDt * kDt * n * (n + 1.0f) * 0.5f);
  Vec3f actual = systemCenterOfMass(world.bodies(), bodies);
  EXPECT_NEAR(glm::length(actual - expected), 0.0f, 1e-3f);
}
