#include <Particle.hpp>
#include <cmath>
#include <gtest/gtest.h>

TEST(ParticleTest, ConstructorSetsFields) {
  Particle p({100.0f, 200.0f}, {10.0f, 20.0f}, 1.0f / 60.0f, 5.0f, 2.0f);
  EXPECT_FLOAT_EQ(p.position.x, 100.0f);
  EXPECT_FLOAT_EQ(p.position.y, 200.0f);
  EXPECT_FLOAT_EQ(p.velocity.x, 10.0f);
  EXPECT_FLOAT_EQ(p.velocity.y, 20.0f);
  EXPECT_FLOAT_EQ(p.radius, 5.0f);
  EXPECT_FLOAT_EQ(p.mass, 2.0f);
  EXPECT_FLOAT_EQ(p.invMass, 0.5f);
}

TEST(ParticleTest, ZeroMassGivesZeroInvMass) {
  Particle p({0.0f, 0.0f}, {0.0f, 0.0f}, 1.0f / 60.0f, 5.0f, 0.0f);
  EXPECT_FLOAT_EQ(p.invMass, 0.0f);
}

TEST(ParticleTest, PrevPositionInitializedFromVelocity) {
  float dt = 1.0f / 60.0f;
  Vec2f pos(100.0f, 200.0f);
  Vec2f vel(60.0f, 120.0f);
  Particle p(pos, vel, dt, 5.0f, 1.0f);
  EXPECT_FLOAT_EQ(p.prevPosition.x, pos.x - vel.x * dt);
  EXPECT_FLOAT_EQ(p.prevPosition.y, pos.y - vel.y * dt);
}

TEST(ParticleTest, UniqueIds) {
  Particle p1({0.0f, 0.0f}, {0.0f, 0.0f});
  Particle p2({0.0f, 0.0f}, {0.0f, 0.0f});
  EXPECT_NE(p1.id, p2.id);
}

TEST(ParticleTest, EulerIntegration) {
  Particle p({0.0f, 0.0f}, {10.0f, 0.0f}, 1.0f / 60.0f, 5.0f, 1.0f);
  p.accelerate({0.0f, 10.0f});
  float dt = 1.0f / 60.0f;
  p.integrateEuler(dt);

  EXPECT_FLOAT_EQ(p.velocity.x, 10.0f);
  EXPECT_FLOAT_EQ(p.velocity.y, 10.0f * dt);
  EXPECT_NEAR(p.position.x, 10.0f * dt, 1e-4f);
  EXPECT_NEAR(p.position.y, 10.0f * dt * dt, 1e-4f);
  EXPECT_FLOAT_EQ(p.acceleration.x, 0.0f);
  EXPECT_FLOAT_EQ(p.acceleration.y, 0.0f);
}

TEST(ParticleTest, VerletIntegration) {
  float dt = 1.0f / 60.0f;
  Particle p({100.0f, 100.0f}, {0.0f, 0.0f}, dt, 5.0f, 1.0f);
  p.accelerate({0.0f, 10.0f});
  p.integrateVerlet(dt);

  EXPECT_FLOAT_EQ(p.prevPosition.x, 100.0f);
  EXPECT_FLOAT_EQ(p.prevPosition.y, 100.0f);
  EXPECT_NEAR(p.position.y, 100.0f + 10.0f * dt * dt, 1e-4f);
  EXPECT_FLOAT_EQ(p.acceleration.x, 0.0f);
  EXPECT_FLOAT_EQ(p.acceleration.y, 0.0f);
}

TEST(ParticleTest, AccelerationAccumulates) {
  Particle p({0.0f, 0.0f}, {0.0f, 0.0f});
  p.accelerate({1.0f, 2.0f});
  p.accelerate({3.0f, 4.0f});
  EXPECT_FLOAT_EQ(p.acceleration.x, 4.0f);
  EXPECT_FLOAT_EQ(p.acceleration.y, 6.0f);
}

TEST(ParticleTest, IntegrationDefinitionsInSourceFile) {
  // After the refactor, Particle integration methods should be declared
  // in include/Particle.hpp but defined in src/Particle.cpp.
  // This test simply ensures the methods are callable and produce correct
  // results.
  float dt = 0.01f;
  Particle p1({50.0f, 50.0f}, {5.0f, 0.0f}, dt, 3.0f, 1.0f);
  p1.accelerate({0.0f, -9.81f});
  p1.integrateEuler(dt);
  EXPECT_GT(p1.position.x, 50.0f);

  Particle p2({50.0f, 50.0f}, {5.0f, 0.0f}, dt, 3.0f, 1.0f);
  p2.accelerate({0.0f, -9.81f});
  p2.integrateVerlet(dt);
  EXPECT_GT(p2.position.x, 50.0f);
}
