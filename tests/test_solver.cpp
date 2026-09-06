#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <core/World.hpp>
#include <dynamics/Solver.hpp>

namespace {

BodyHandle addSphere(BodyStore& store, Vec3f position, float invMass,
                     float radius = 1.0f) {
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{radius}};
  desc.transform.position = position;
  desc.invMass = invMass;
  desc.constrainTo2D = true;
  return store.addBody(desc);
}

Manifold buildSphereManifold(BodyStore& store, BodyHandle a, BodyHandle b) {
  Transform ta = store.transform(a);
  Transform tb = store.transform(b);
  GjkResult gjk = gjkOverlap(store.shape(a), ta, store.shape(b), tb);
  EpaResult epa = epaPenetration(store.shape(a), ta, store.shape(b), tb, gjk);
  return buildManifold(store.shape(a), ta, store.shape(b), tb, a, b, gjk, epa);
}

Quatf rotationAboutZ(float radians) {
  return Quatf(std::cos(radians * 0.5f), 0.0f, 0.0f, std::sin(radians * 0.5f));
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

TEST(SolverTest, RestitutionProducesExpectedBounceHeight) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));

  Transform groundT;
  groundT.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 0.0f)}},
                         groundT, /*friction=*/0.0f, /*restitution=*/0.0f,
                         /*constrainTo2D=*/true);

  const float restitution = 0.8f;
  const float h0 = 5.0f;
  const float sphereRadius = 0.5f;
  Transform sphereT;
  sphereT.position = Vec3f(0.0f, sphereRadius + h0, 0.0f);
  BodyHandle sphere = world.createDynamicBody(
      ShapeVariant{SphereShape{sphereRadius}}, sphereT, /*mass=*/1.0f,
      /*friction=*/0.0f, restitution, /*constrainTo2D=*/true);

  const float dt = 1.0f / 240.0f;
  const float restY = sphereRadius;
  bool wasFalling = false;
  bool bounced = false;
  float apexY = restY;

  for (int i = 0; i < 240 * 6; i++) {
    world.step(dt);
    float y = world.bodies().position(sphere).y;
    float vy = world.bodies().linearVelocity(sphere).y;

    if (!bounced) {
      if (wasFalling && vy > 0.0f) { bounced = true; }
      wasFalling = vy < 0.0f;
    } else {
      if (vy <= 0.0f) { break; }
      apexY = std::max(apexY, y);
    }
  }

  ASSERT_TRUE(bounced);
  float bounceHeight = apexY - restY;
  float expectedBounceHeight = restitution * restitution * h0;
  EXPECT_NEAR(bounceHeight, expectedBounceHeight,
              expectedBounceHeight * 0.3f + 0.1f);
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

  Vec3f localOffset(0.0f, 0.99f, 0.0f);
  Vec3f worldOffset = rampOrientation * localOffset;
  Transform boxT;
  boxT.position = worldOffset;
  boxT.orientation = rampOrientation;
  BodyHandle box = world.createDynamicBody(
      ShapeVariant{BoxShape{Vec3f(0.5f, 0.5f, 0.0f)}}, boxT, /*mass=*/1.0f,
      friction, /*restitution=*/0.0f, /*constrainTo2D=*/true);

  Vec3f tangent(std::cos(inclineRadians), std::sin(inclineRadians), 0.0f);
  const float dt = 1.0f / 120.0f;
  for (int i = 0; i < 600; i++) { world.step(dt); }

  return glm::dot(world.bodies().linearVelocity(box), tangent);
}
}  // namespace

TEST(SolverTest, FrictionStopsSlidingAboveCriticalCoefficient) {
  const float inclineRadians = 0.4636f;  // ~26.57 deg, tan == 0.5
  float speedWithHighFriction = finalTangentialSpeed(1.0f, inclineRadians);
  EXPECT_NEAR(speedWithHighFriction, 0.0f, 0.5f);
}

TEST(SolverTest, ZeroFrictionKeepsAcceleratingDownTheSlope) {
  const float inclineRadians = 0.4636f;
  float speedWithNoFriction = finalTangentialSpeed(0.0f, inclineRadians);
  EXPECT_LT(speedWithNoFriction, -1.0f);
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
