#include <Simulator.hpp>
#include <gtest/gtest.h>
#include <cmath>

class CollisionMathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sim = std::make_unique<Simulator>(
        Vec2f(800.0f, 600.0f), 20.0f, 0.0f, 1.0f, 1.0f / 60.0f, 10000,
        IntegrationType::Euler, BroadphaseType::Naive);
  }
  std::unique_ptr<Simulator> sim;
};

TEST_F(CollisionMathTest, TwoCollidingParticlesSeparate) {
  sim->spawnParticle({100.0f, 300.0f}, {50.0f, 0.0f}, 10.0f, 1.0f);
  sim->spawnParticle({115.0f, 300.0f}, {-50.0f, 0.0f}, 10.0f, 1.0f);

  sim->update();

  const auto& p1 = sim->particles()[0];
  const auto& p2 = sim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  float sumR = p1.radius + p2.radius;

  EXPECT_GE(dist, sumR * 0.95f);
}

TEST_F(CollisionMathTest, MomentumConservedInElasticCollision) {
  auto elasticSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 0.0f, 1.0f, 1.0f / 60.0f, 10000,
      IntegrationType::Euler, BroadphaseType::Naive);

  elasticSim->spawnParticle({200.0f, 300.0f}, {50.0f, 0.0f}, 10.0f, 1.0f);
  elasticSim->spawnParticle({225.0f, 300.0f}, {-50.0f, 0.0f}, 10.0f, 1.0f);

  const auto& particles = elasticSim->particles();
  float initialMomentumX = particles[0].mass * particles[0].velocity.x +
                           particles[1].mass * particles[1].velocity.x;

  elasticSim->update();

  float finalMomentumX = particles[0].mass * particles[0].velocity.x +
                         particles[1].mass * particles[1].velocity.x;

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 5.0f);
}

TEST_F(CollisionMathTest, NoCollisionWhenFarApart) {
  sim->spawnParticle({100.0f, 300.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);
  sim->spawnParticle({500.0f, 300.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);

  float x1Before = sim->particles()[0].position.x;
  float x2Before = sim->particles()[1].position.x;

  sim->update();

  EXPECT_FLOAT_EQ(sim->particles()[0].position.x, x1Before);
  EXPECT_FLOAT_EQ(sim->particles()[1].position.x, x2Before);
}

TEST_F(CollisionMathTest, NearZeroDistanceHandledSafely) {
  sim->spawnParticle({300.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);
  sim->spawnParticle({300.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);

  EXPECT_NO_THROW({
    for (int i = 0; i < 10; i++) {
      sim->update();
    }
  });

  const auto& p1 = sim->particles()[0];
  const auto& p2 = sim->particles()[1];
  EXPECT_FALSE(std::isnan(p1.position.x));
  EXPECT_FALSE(std::isnan(p1.position.y));
  EXPECT_FALSE(std::isnan(p2.position.x));
  EXPECT_FALSE(std::isnan(p2.position.y));
  EXPECT_FALSE(std::isinf(p1.position.x));
  EXPECT_FALSE(std::isinf(p2.position.x));
}

TEST_F(CollisionMathTest, UnequalMassCollision) {
  sim->spawnParticle({200.0f, 300.0f}, {20.0f, 0.0f}, 10.0f, 5.0f);
  sim->spawnParticle({225.0f, 300.0f}, {-10.0f, 0.0f}, 10.0f, 1.0f);

  sim->update();

  const auto& heavy = sim->particles()[0];
  const auto& light = sim->particles()[1];
  EXPECT_GT(heavy.velocity.x, -20.0f);
  EXPECT_GT(light.velocity.x, -10.0f);
}

TEST_F(CollisionMathTest, PositionalCorrectionResolvesOverlap) {
  sim->spawnParticle({300.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);
  sim->spawnParticle({310.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);

  for (int i = 0; i < 50; i++) {
    sim->update();
  }

  const auto& p1 = sim->particles()[0];
  const auto& p2 = sim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  EXPECT_GE(dist, p1.radius + p2.radius - 1.0f);
}

TEST_F(CollisionMathTest, VerletCollisionResolution) {
  auto verletSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 0.0f, 0.5f, 1.0f / 60.0f, 10000,
      IntegrationType::Verlet, BroadphaseType::Naive);

  verletSim->spawnParticle({200.0f, 300.0f}, {50.0f, 0.0f}, 10.0f, 1.0f);
  verletSim->spawnParticle({215.0f, 300.0f}, {-50.0f, 0.0f}, 10.0f, 1.0f);

  verletSim->update();

  const auto& p1 = verletSim->particles()[0];
  const auto& p2 = verletSim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  EXPECT_GE(dist, p1.radius + p2.radius - 2.0f);
}
