#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <core/World.hpp>
#include <dynamics/Solver.hpp>

#include "test_helpers.hpp"

using test_helpers::buildManifoldBetween;
using test_helpers::rotationAboutAxis;

namespace {

BodyHandle addSphere(BodyStore& store, Vec3f position, float invMass,
                     float radius = 1.0f) {
  return test_helpers::spawnSphere(store, position, invMass, radius,
                                   Vec3f(0.0f), /*constrainTo2D=*/true);
}

Manifold buildSphereManifold(BodyStore& store, BodyHandle a, BodyHandle b) {
  return buildManifoldBetween(store, a, b);
}

Quatf rotationAboutZ(float radians) {
  return rotationAboutAxis(Vec3f(0.0f, 0.0f, 1.0f), radians);
}

}  // namespace

TEST(SolverTest, WarmStartReducesSecondFrameFirstIterationDeltaImpulse) {
  BodyStore store;
  BodyHandle a = addSphere(store, Vec3f(0.0f), 0.0f);              // static
  BodyHandle b = addSphere(store, Vec3f(1.5f, 0.0f, 0.0f), 1.0f);  // dynamic

  Manifold manifold = buildSphereManifold(store, a, b);
  ASSERT_GE(manifold.pointCount, 1);
  std::array<Manifold, 1> manifolds{manifold};

  Vec3f closingVelocity(-2.0f, 0.0f, 0.0f);
  store.linearVelocity(b) = closingVelocity;

  std::vector<float> bias1 = prepareRestitutionBias(manifolds, store);
  warmStart(manifolds, store);  // no-op as normalImpulse starts at 0
  float before1 = manifolds[0].points[0].normalImpulse;
  solveVelocity(manifolds, store, bias1);
  float firstDelta = manifolds[0].points[0].normalImpulse - before1;
  for (int i = 1; i < 8; i++) { solveVelocity(manifolds, store, bias1); }

  store.linearVelocity(b) = closingVelocity;
  std::vector<float> bias2 = prepareRestitutionBias(manifolds, store);
  warmStart(manifolds, store);
  float before2 = manifolds[0].points[0].normalImpulse;
  solveVelocity(manifolds, store, bias2);
  float secondDelta = manifolds[0].points[0].normalImpulse - before2;

  EXPECT_LT(std::abs(secondDelta), std::abs(firstDelta));
}

TEST(SolverTest, SolvePositionSeparatesADeeplyPenetratingPair) {
  BodyStore store;
  BodyHandle a = addSphere(store, Vec3f(0.0f), 0.0f);              // static
  BodyHandle b = addSphere(store, Vec3f(1.5f, 0.0f, 0.0f), 1.0f);  // overlap .5

  Manifold manifold = buildSphereManifold(store, a, b);
  ASSERT_GE(manifold.pointCount, 1);
  std::array<Manifold, 1> manifolds{manifold};

  SolverConfig config;
  float xBefore = store.position(b).x;
  solvePosition(manifolds, store, config);
  float xAfter = store.position(b).x;

  EXPECT_GT(xAfter, xBefore + 0.01f);
}

TEST(SolverTest, SolveVelocityUsesManifoldNormalNotRawPositionDelta) {
  BodyStore store;
  BodyHandle a = addSphere(store, Vec3f(0.0f), 0.0f);               // static
  BodyHandle b = addSphere(store, Vec3f(10.0f, 0.0f, 0.0f), 1.0f);  // dynamic

  store.linearVelocity(b) = Vec3f(0.0f, -3.0f, 0.0f);

  Manifold manifold;
  manifold.bodyA = a;
  manifold.bodyB = b;
  manifold.normal = Vec3f(0.0f, 1.0f, 0.0f);
  manifold.pointCount = 1;
  manifold.points[0] = ManifoldPoint{
      .localAnchorA = Vec3f(0.0f, 0.0f, 0.0f),
      .localAnchorB = Vec3f(0.0f, -0.1f, 0.0f),
      .penetration = 0.1f,
  };
  std::array<Manifold, 1> manifolds{manifold};

  std::vector<float> bias = prepareRestitutionBias(manifolds, store);
  warmStart(manifolds, store);  // no-op, impulses start at 0
  solveVelocity(manifolds, store, bias);

  EXPECT_NEAR(store.linearVelocity(b).x, 0.0f, 1e-4f);
  EXPECT_NEAR(store.linearVelocity(b).y, 0.0f, 1e-4f);
  EXPECT_NEAR(store.linearVelocity(b).z, 0.0f, 1e-4f);

  float xBefore = store.position(b).x;
  SolverConfig config;
  solvePosition(manifolds, store, config);

  EXPECT_NEAR(store.position(b).x, xBefore, 1e-5f);
  EXPECT_GT(store.position(b).y, 0.01f);
}

namespace {
float finalTangentialSpeed(float friction, float inclineRadians) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));

  Quatf rampOrientation = rotationAboutZ(inclineRadians);
  Transform rampT;
  rampT.orientation = rampOrientation;
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 0.0f)}},
                         rampT, friction, /*restitution=*/0.0f,
                         /*constrainTo2D=*/true);

  constexpr float kHalfWidth = 1.0f;
  constexpr float kHalfHeight = 0.2f;
  Vec3f localOffset(0.0f, 0.5f + kHalfHeight, 0.0f);
  Vec3f worldOffset = rampOrientation * localOffset;
  Transform boxT;
  boxT.position = worldOffset;
  boxT.orientation = rampOrientation;
  BodyHandle box = world.createDynamicBody(
      ShapeVariant{BoxShape{Vec3f(kHalfWidth, kHalfHeight, 0.0f)}}, boxT,
      /*mass=*/1.0f, friction, /*restitution=*/0.0f, /*constrainTo2D=*/true);

  Vec3f tangent(std::cos(inclineRadians), std::sin(inclineRadians), 0.0f);
  const float dt = 1.0f / 120.0f;
  for (int i = 0; i < 600; i++) { world.step(dt); }

  return glm::dot(world.bodies().linearVelocity(box), tangent);
}
}  // namespace

TEST(SolverTest, FrictionConeBoundaryMatchesAnalyticSlopeAngle) {
  const float g = 9.81f;
  const float durationSeconds = 600.0f * (1.0f / 120.0f);

  for (float mu : {0.5f, 1.0f}) {
    float criticalAngle = std::atan(mu);

    {
      SCOPED_TRACE("mu=" + std::to_string(mu) + " just below critical");
      float angle = criticalAngle - 0.05f;
      float speed = finalTangentialSpeed(mu, angle);
      EXPECT_NEAR(speed, 0.0f, 0.05f);
    }
    {
      SCOPED_TRACE("mu=" + std::to_string(mu) + " just above critical");
      float angle = criticalAngle + 0.05f;
      float speed = finalTangentialSpeed(mu, angle);
      float expectedSpeed =
          -g * (std::sin(angle) - mu * std::cos(angle)) * durationSeconds;
      EXPECT_NEAR(speed, expectedSpeed, 0.1f * std::abs(expectedSpeed));
    }
  }
}

TEST(SolverTest, StackOfBoxesRemainsStable) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));

  Transform groundT;
  groundT.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 0.0f)}},
                         groundT, /*friction=*/0.8f, /*restitution=*/0.0f,
                         /*constrainTo2D=*/true);

  constexpr int kBoxCount = 5;
  constexpr float kHalfExtent = 0.5f;
  std::array<BodyHandle, kBoxCount> boxes{};
  for (int i = 0; i < kBoxCount; i++) {
    Transform t;
    t.position =
        Vec3f(0.0f, kHalfExtent + i * (2.0f * kHalfExtent - 0.002f), 0.0f);
    boxes[i] = world.createDynamicBody(
        ShapeVariant{BoxShape{Vec3f(kHalfExtent, kHalfExtent, 0.0f)}}, t,
        /*mass=*/1.0f, /*friction=*/0.8f, /*restitution=*/0.0f,
        /*constrainTo2D=*/true);
  }

  const float dt = 1.0f / 120.0f;
  for (int i = 0; i < 300; i++) { world.step(dt); }

  for (int i = 0; i < kBoxCount; i++) {
    Vec3f pos = world.bodies().position(boxes[i]);
    ASSERT_FALSE(std::isnan(pos.x));
    ASSERT_FALSE(std::isnan(pos.y));
    float expectedY = kHalfExtent + i * (2.0f * kHalfExtent);
    EXPECT_NEAR(pos.y, expectedY, 0.15f) << "box " << i;
    EXPECT_NEAR(pos.x, 0.0f, 0.2f) << "box " << i;

    Quatf orientation = world.bodies().orientation(boxes[i]);
    EXPECT_GT(std::abs(orientation.w), 0.97f) << "box " << i << " tipped over";
  }
}

TEST(SolverTest, StaticEquilibriumHasZeroDrift) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));

  Transform groundT;
  groundT.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 0.0f)}},
                         groundT, /*friction=*/0.8f, /*restitution=*/0.0f,
                         /*constrainTo2D=*/true);

  constexpr float kHalfExtent = 0.5f;
  Transform boxT;
  boxT.position = Vec3f(0.0f, kHalfExtent + 0.1f, 0.0f);
  BodyHandle box = world.createDynamicBody(
      ShapeVariant{BoxShape{Vec3f(kHalfExtent, kHalfExtent, 0.0f)}}, boxT,
      /*mass=*/1.0f, /*friction=*/0.8f, /*restitution=*/0.0f,
      /*constrainTo2D=*/true);

  const float dt = 1.0f / 120.0f;
  constexpr int kSettleSteps = 2000;
  constexpr int kMeasureSteps = 500;
  for (int i = 0; i < kSettleSteps; i++) { world.step(dt); }

  std::vector<double> ys;
  ys.reserve(kMeasureSteps);
  for (int i = 0; i < kMeasureSteps; i++) {
    world.step(dt);
    ys.push_back(static_cast<double>(world.bodies().position(box).y));
  }

  double mean = 0.0;
  for (double y : ys) { mean += y; }
  mean /= static_cast<double>(ys.size());

  double variance = 0.0;
  for (double y : ys) { variance += (y - mean) * (y - mean); }
  variance /= static_cast<double>(ys.size());

  EXPECT_LT(variance, 1e-9);
}

TEST(SolverTest, InertiaTensorMatchesParallelAxisTheorem) {
  const float r = 0.1f;
  const float pointMass = 1.0f;
  const float d = 2.0f;
  SphereShape sphere{r};
  Mat3f localInertia = sphere.localInertiaTensor(pointMass);

  Mat3f inertia(0.0f);
  inertia[0][0] = 2.0f * localInertia[0][0];
  inertia[1][1] = 2.0f * (localInertia[1][1] + pointMass * d * d);
  inertia[2][2] = 2.0f * (localInertia[2][2] + pointMass * d * d);
  Mat3f invInertia = glm::inverse(inertia);

  BodyStore store;
  BodyDesc staticDesc;
  staticDesc.shape = ShapeVariant{SphereShape{0.1f}};
  BodyHandle a = store.addBody(staticDesc);

  BodyDesc dumbbellDesc;
  dumbbellDesc.shape = ShapeVariant{SphereShape{r}};
  dumbbellDesc.invMass = 1.0f / (2.0f * pointMass);
  dumbbellDesc.invInertiaBody = invInertia;
  BodyHandle b = store.addBody(dumbbellDesc);

  Mat3f roundTrip = store.invInertiaBody(b);
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 3; row++) {
      EXPECT_NEAR(roundTrip[col][row], invInertia[col][row], 1e-6f);
    }
  }

  store.linearVelocity(b) = Vec3f(-2.0f, 0.0f, 0.0f);

  Manifold manifold;
  manifold.bodyA = a;
  manifold.bodyB = b;
  manifold.normal = Vec3f(1.0f, 0.0f, 0.0f);
  manifold.pointCount = 1;
  manifold.points[0] = ManifoldPoint{
      .localAnchorA = Vec3f(0.0f, 0.0f, 0.0f),
      .localAnchorB = Vec3f(0.0f, 1.0f, 0.0f),
      .penetration = 0.1f,
  };
  std::array<Manifold, 1> manifolds{manifold};

  std::vector<float> bias = prepareRestitutionBias(manifolds, store);
  solveVelocity(manifolds, store, bias);

  float appliedNormalImpulse = manifolds[0].points[0].normalImpulse;
  Vec3f impulseVec = manifolds[0].normal * appliedNormalImpulse;
  Vec3f rB = manifolds[0].points[0].localAnchorB;
  Vec3f expectedDeltaOmega = invInertia * glm::cross(rB, impulseVec);

  EXPECT_NEAR(store.angularVelocity(b).x, expectedDeltaOmega.x, 1e-4f);
  EXPECT_NEAR(store.angularVelocity(b).y, expectedDeltaOmega.y, 1e-4f);
  EXPECT_NEAR(store.angularVelocity(b).z, expectedDeltaOmega.z, 1e-4f);
}
