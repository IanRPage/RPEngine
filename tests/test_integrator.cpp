#include <gtest/gtest.h>

#include <core/BodyStore.hpp>
#include <dynamics/Integrator.hpp>

namespace {
BodyHandle addDynamicBody(BodyStore& store, Vec3f position,
                          bool constrainTo2D = false) {
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{1.0f}};
  desc.transform.position = position;
  desc.invMass = 1.0f;
  desc.constrainTo2D = constrainTo2D;
  return store.addBody(desc);
}

BodyHandle addStaticBody(BodyStore& store, Vec3f position) {
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{1.0f}};
  desc.transform.position = position;
  desc.invMass = 0.0f;
  return store.addBody(desc);
}
}  // namespace

TEST(IntegratorTest, GravityAccumulatesLinearlyOverNSteps) {
  BodyStore store;
  BodyHandle h = addDynamicBody(store, Vec3f(0.0f));
  Vec3f gravity(0.0f, -9.81f, 0.0f);
  float dt = 1.0f / 60.0f;

  for (int i = 0; i < 10; i++) integrateVelocity(store, dt, gravity);

  EXPECT_NEAR(store.linearVelocity(h).y, gravity.y * dt * 10, 1e-4f);
}

TEST(IntegratorTest, StaticBodyIsUnaffectedByGravityOrIntegration) {
  BodyStore store;
  BodyHandle h = addStaticBody(store, Vec3f(5.0f, 5.0f, 0.0f));

  integrateVelocity(store, 1.0f / 60.0f, Vec3f(0.0f, -9.81f, 0.0f));
  integratePosition(store, 1.0f / 60.0f);

  EXPECT_FLOAT_EQ(store.linearVelocity(h).y, 0.0f);
  EXPECT_FLOAT_EQ(store.position(h).x, 5.0f);
  EXPECT_FLOAT_EQ(store.position(h).y, 5.0f);
}

TEST(IntegratorTest, PositionIntegratesFromVelocitySemiImplicitEuler) {
  BodyStore store;
  BodyHandle h = addDynamicBody(store, Vec3f(0.0f));
  store.linearVelocity(h) = Vec3f(2.0f, 0.0f, 0.0f);

  integratePosition(store, 0.5f);

  EXPECT_FLOAT_EQ(store.position(h).x, 1.0f);
}

TEST(IntegratorTest, ConstrainedBodyStaysInPlaneAndLosesOutOfPlaneSpin) {
  BodyStore store;
  BodyHandle h = addDynamicBody(store, Vec3f(0.0f, 0.0f, 3.0f), true);
  store.linearVelocity(h) = Vec3f(1.0f, 0.0f, 1.0f);
  store.angularVelocity(h) = Vec3f(1.0f, 1.0f, 1.0f);

  integratePosition(store, 1.0f / 60.0f);

  EXPECT_FLOAT_EQ(store.position(h).z, 0.0f);
  EXPECT_FLOAT_EQ(store.linearVelocity(h).z, 0.0f);
  EXPECT_FLOAT_EQ(store.angularVelocity(h).x, 0.0f);
  EXPECT_FLOAT_EQ(store.angularVelocity(h).y, 0.0f);
  // the one rotational DoF 2D body keeps
  EXPECT_FLOAT_EQ(store.angularVelocity(h).z, 1.0f);
}

TEST(IntegratorTest, SemiImplicitEulerIntegratorMatchesFreeFunctions) {
  BodyStore viaInterface;
  BodyStore viaFreeFunctions;
  BodyHandle a = addDynamicBody(viaInterface, Vec3f(0.0f));
  BodyHandle b = addDynamicBody(viaFreeFunctions, Vec3f(0.0f));

  SemiImplicitEulerIntegrator integrator;
  Vec3f gravity(0.0f, -9.81f, 0.0f);
  float dt = 1.0f / 60.0f;

  integrator.integrateVelocity(viaInterface, dt, gravity);
  integrator.integratePosition(viaInterface, dt);

  integrateVelocity(viaFreeFunctions, dt, gravity);
  integratePosition(viaFreeFunctions, dt);

  EXPECT_FLOAT_EQ(viaInterface.position(a).y, viaFreeFunctions.position(b).y);
  EXPECT_FLOAT_EQ(viaInterface.linearVelocity(a).y,
                 viaFreeFunctions.linearVelocity(b).y);
}
