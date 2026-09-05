#include <Simulator.hpp>
#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>

class ConstraintsTest : public ::testing::Test {
protected:
  void SetUp() override {
    sim = std::make_unique<Simulator>(
        Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 10000,
        IntegrationType::Verlet, BroadphaseType::Naive);
  }
  std::unique_ptr<Simulator> sim;
};

TEST_F(ConstraintsTest, ContactConstraintResolvesPenetration) {
  sim->spawnParticle({300.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);
  sim->spawnParticle({312.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);

  for (int i = 0; i < 100; i++) {
    sim->update();
  }

  const auto &p1 = sim->particles()[0];
  const auto &p2 = sim->particles()[1];
  float dx = p2.position.x - p1.position.x;
  float dy = p2.position.y - p1.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  EXPECT_GE(dist, p1.radius + p2.radius - 1.0f);
}

TEST_F(ConstraintsTest, WallConstraintKeepsParticlesInBounds) {
  sim->spawnParticle({5.0f, 5.0f}, {-100.0f, -100.0f}, 5.0f, 1.0f);
  sim->spawnParticle({795.0f, 595.0f}, {100.0f, 100.0f}, 5.0f, 1.0f);

  for (int i = 0; i < 100; i++) {
    sim->update();
  }

  for (const auto &p : sim->particles()) {
    EXPECT_GE(p.position.x, p.radius - 0.5f);
    EXPECT_LE(p.position.x, 800.0f - p.radius + 0.5f);
    EXPECT_GE(p.position.y, p.radius - 0.5f);
    EXPECT_LE(p.position.y, 600.0f - p.radius + 0.5f);
  }
}

TEST_F(ConstraintsTest, ConstraintsWorkWithDifferentMasses) {
  sim->spawnParticle({300.0f, 300.0f}, {20.0f, 0.0f}, 10.0f, 10.0f);
  sim->spawnParticle({315.0f, 300.0f}, {-5.0f, 0.0f}, 10.0f, 0.5f);

  for (int i = 0; i < 100; i++) {
    sim->update();
  }

  const auto &p1 = sim->particles()[0];
  const auto &p2 = sim->particles()[1];
  EXPECT_FALSE(std::isnan(p1.position.x));
  EXPECT_FALSE(std::isnan(p2.position.x));
}

TEST_F(ConstraintsTest, MultiBodyPileupDoesNotExplode) {
  for (int i = 0; i < 15; i++) {
    sim->spawnParticle({400.0f, 590.0f - i * 3.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);
  }

  float maxSpeedSq = 0.0f;
  for (int frame = 0; frame < 1000; frame++) {
    sim->update();
    for (const auto &p : sim->particles()) {
      Vec2f v = p.position - p.prevPosition;
      float speedSq = v.x * v.x + v.y * v.y;
      maxSpeedSq = std::max(maxSpeedSq, speedSq);
    }
  }

  EXPECT_LT(std::sqrt(maxSpeedSq), 50.0f)
      << "Pileup should not cause explosive velocities";
}
