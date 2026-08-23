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

  for (int i = 0; i < 50; i++) {
    sim->update();
  }

  const auto& p1 = sim->particles()[0];
  const auto& p2 = sim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  float sumR = p1.radius + p2.radius;

  EXPECT_GE(dist, sumR - 1.0f);

  EXPECT_NEAR(p1.velocity.x, -50.0f, 1.0f);
  EXPECT_NEAR(p2.velocity.x, 50.0f, 1.0f);
}

TEST_F(CollisionMathTest, MomentumConservedInElasticCollision) {
  auto elasticSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 0.0f, 1.0f, 1.0f / 60.0f, 10000,
      IntegrationType::Euler, BroadphaseType::Naive);

  elasticSim->spawnParticle({200.0f, 300.0f}, {50.0f, 0.0f}, 10.0f, 1.0f);
  elasticSim->spawnParticle({225.0f, 300.0f}, {-50.0f, 0.0f}, 10.0f, 1.0f);

  const auto& particles = elasticSim->particles();
  const float m0 = particles[0].mass;
  const float m1 = particles[1].mass;
  float initialV0 = particles[0].velocity.x;
  float initialMomentumX =
      m0 * particles[0].velocity.x + m1 * particles[1].velocity.x;
  float initialKE = 0.5f * m0 * particles[0].velocity.x * particles[0].velocity.x +
                    0.5f * m1 * particles[1].velocity.x * particles[1].velocity.x;

  for (int i = 0; i < 20; i++) {
    elasticSim->update();
  }

  float finalMomentumX =
      m0 * particles[0].velocity.x + m1 * particles[1].velocity.x;
  float finalKE = 0.5f * m0 * particles[0].velocity.x * particles[0].velocity.x +
                  0.5f * m1 * particles[1].velocity.x * particles[1].velocity.x;

  ASSERT_GT(std::fabs(particles[0].velocity.x - initialV0), 10.0f)
      << "Particles never collided; momentum conservation wasn't exercised";

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 1.0f);
  EXPECT_NEAR(initialKE, finalKE, 5.0f)
      << "Restitution is 1.0 (perfectly elastic); kinetic energy should be "
         "conserved";
}

TEST_F(CollisionMathTest, NoCollisionWhenFarApart) {
  sim->spawnParticle({100.0f, 300.0f}, {0.01f, 0.0f}, 5.0f, 1.0f);
  sim->spawnParticle({500.0f, 300.0f}, {-0.01f, 0.0f}, 5.0f, 1.0f);

  float distBefore =
      sim->particles()[1].position.x - sim->particles()[0].position.x;

  sim->update();

  float distAfter =
      sim->particles()[1].position.x - sim->particles()[0].position.x;

  EXPECT_NEAR(distAfter, distBefore, 0.01f);
}

TEST_F(CollisionMathTest, NearZeroDistanceHandledSafely) {
  sim->spawnParticle({300.0f, 300.0f}, {5.0f, 5.0f}, 10.0f, 1.0f);
  sim->spawnParticle({300.0f, 300.0f}, {5.0f, 5.0f}, 10.0f, 1.0f);

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

  const float m0 = sim->particles()[0].mass;
  const float m1 = sim->particles()[1].mass;
  float initialMomentumX =
      m0 * sim->particles()[0].velocity.x + m1 * sim->particles()[1].velocity.x;
  float initialKE =
      0.5f * m0 * sim->particles()[0].velocity.x * sim->particles()[0].velocity.x +
      0.5f * m1 * sim->particles()[1].velocity.x * sim->particles()[1].velocity.x;

  for (int i = 0; i < 20; i++) {
    sim->update();
  }

  const auto& heavy = sim->particles()[0];
  const auto& light = sim->particles()[1];

  EXPECT_GT(heavy.velocity.x, 0.0f);
  EXPECT_LT(heavy.velocity.x, 20.0f);
  EXPECT_GT(light.velocity.x, 0.0f);

  float finalMomentumX = m0 * heavy.velocity.x + m1 * light.velocity.x;
  float finalKE = 0.5f * m0 * heavy.velocity.x * heavy.velocity.x +
                  0.5f * m1 * light.velocity.x * light.velocity.x;

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 1.0f);
  EXPECT_NEAR(initialKE, finalKE, 5.0f)
      << "Restitution is 1.0 (perfectly elastic); kinetic energy should be "
         "conserved";
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

  const float dt = 1.0f / 60.0f;
  verletSim->spawnParticle({200.0f, 300.0f}, {50.0f, 0.0f}, 10.0f, 1.0f);
  verletSim->spawnParticle({215.0f, 300.0f}, {-50.0f, 0.0f}, 10.0f, 1.0f);

  const float m0 = verletSim->particles()[0].mass;
  const float m1 = verletSim->particles()[1].mass;
  float initialV0 = verletSim->particles()[0].velocity.x;
  float initialV1 = verletSim->particles()[1].velocity.x;
  float initialMomentumX = m0 * initialV0 + m1 * initialV1;
  float initialKE =
      0.5f * m0 * initialV0 * initialV0 + 0.5f * m1 * initialV1 * initialV1;

  for (int i = 0; i < 50; i++) {
    verletSim->update();
  }

  const auto& p1 = verletSim->particles()[0];
  const auto& p2 = verletSim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  EXPECT_GE(dist, p1.radius + p2.radius - 1.0f);

  float finalV0 = (p1.position.x - p1.prevPosition.x) / dt;
  float finalV1 = (p2.position.x - p2.prevPosition.x) / dt;
  float finalMomentumX = m0 * finalV0 + m1 * finalV1;
  float finalKE = 0.5f * m0 * finalV0 * finalV0 + 0.5f * m1 * finalV1 * finalV1;

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 2.0f)
      << "No gravity or wall contact in this scenario; momentum should be "
         "conserved regardless of restitution";
  EXPECT_LE(finalKE, initialKE + 5.0f)
      << "Restitution is 0.5 (inelastic); kinetic energy should not "
         "increase, but was "
      << finalKE << " (initial was " << initialKE << ")";
}
