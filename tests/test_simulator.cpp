#include <Simulator.hpp>
#include <gtest/gtest.h>
#include <cmath>

class SimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sim = std::make_unique<Simulator>(
        Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 10000,
        IntegrationType::Verlet, BroadphaseType::Naive);
  }
  std::unique_ptr<Simulator> sim;
};

TEST_F(SimulatorTest, InitialStateNoParticles) {
  EXPECT_EQ(sim->particles().size(), 0u);
}

TEST_F(SimulatorTest, SpawnParticleAddsParticle) {
  sim->spawnParticle({400.0f, 300.0f}, {10.0f, 0.0f}, 5.0f, 1.0f);
  EXPECT_EQ(sim->particles().size(), 1u);
}

TEST_F(SimulatorTest, SpawnParticleRespectsCapacity) {
  auto smallSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 3);
  smallSim->spawnParticle({100.0f, 100.0f}, {1.0f, 0.0f});
  smallSim->spawnParticle({200.0f, 100.0f}, {1.0f, 0.0f});
  smallSim->spawnParticle({300.0f, 100.0f}, {1.0f, 0.0f});
  smallSim->spawnParticle({400.0f, 100.0f}, {1.0f, 0.0f});
  EXPECT_EQ(smallSim->particles().size(), 3u);
}

TEST_F(SimulatorTest, UpdateAppliesGravity) {
  sim->spawnParticle({400.0f, 100.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);
  float initialY = sim->particles()[0].position.y;
  sim->update();
  EXPECT_GT(sim->particles()[0].position.y, initialY);
}

TEST_F(SimulatorTest, ParticlesStayWithinBounds) {
  sim->spawnParticle({400.0f, 590.0f}, {0.0f, 100.0f}, 5.0f, 1.0f);
  for (int i = 0; i < 100; i++) {
    sim->update();
  }
  const auto& p = sim->particles()[0];
  EXPECT_GE(p.position.x, p.radius);
  EXPECT_LE(p.position.x, 800.0f - p.radius);
  EXPECT_GE(p.position.y, p.radius);
  EXPECT_LE(p.position.y, 600.0f - p.radius);
}

TEST_F(SimulatorTest, WallBounceWithRestitution) {
  auto eulerSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 0.0f, 0.5f, 1.0f / 60.0f, 10000,
      IntegrationType::Euler, BroadphaseType::Naive);
  eulerSim->spawnParticle({5.0f, 300.0f}, {-100.0f, 0.0f}, 5.0f, 1.0f);
  eulerSim->update();
  const auto& p = eulerSim->particles()[0];
  EXPECT_GE(p.position.x, p.radius);
}

TEST_F(SimulatorTest, SettersAndGetters) {
  sim->setWorldSize({1024.0f, 768.0f});
  EXPECT_FLOAT_EQ(sim->worldSize().x, 1024.0f);
  EXPECT_FLOAT_EQ(sim->worldSize().y, 768.0f);

  sim->setDeltaTime(1.0f / 120.0f);

  sim->setIntegrationType(IntegrationType::Euler);
  EXPECT_EQ(sim->integrationType(), IntegrationType::Euler);

  sim->setBroadphaseType(BroadphaseType::SpatialGrid);
  EXPECT_EQ(sim->broadphaseType(), BroadphaseType::SpatialGrid);
}

TEST_F(SimulatorTest, AllBroadphasesProduceConsistentResults) {
  auto makeSim = [](BroadphaseType bp) {
    auto s = std::make_unique<Simulator>(
        Vec2f(200.0f, 200.0f), 10.0f, 0.0f, 0.5f, 1.0f / 60.0f, 1000,
        IntegrationType::Verlet, bp);
    s->spawnParticle({50.0f, 100.0f}, {30.0f, 0.0f}, 10.0f, 1.0f);
    s->spawnParticle({70.0f, 100.0f}, {-30.0f, 0.0f}, 10.0f, 1.0f);
    for (int i = 0; i < 10; i++) s->update();
    return s;
  };

  auto simNaive = makeSim(BroadphaseType::Naive);
  auto simQtree = makeSim(BroadphaseType::Qtree);
  auto simGrid = makeSim(BroadphaseType::SpatialGrid);

  for (size_t i = 0; i < simNaive->particles().size(); i++) {
    EXPECT_NEAR(simNaive->particles()[i].position.x,
                simQtree->particles()[i].position.x, 1.0f);
    EXPECT_NEAR(simNaive->particles()[i].position.y,
                simQtree->particles()[i].position.y, 1.0f);
    EXPECT_NEAR(simNaive->particles()[i].position.x,
                simGrid->particles()[i].position.x, 1.0f);
    EXPECT_NEAR(simNaive->particles()[i].position.y,
                simGrid->particles()[i].position.y, 1.0f);
  }
}

TEST_F(SimulatorTest, EulerAndVerletBothFunctional) {
  auto eulerSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 1000,
      IntegrationType::Euler, BroadphaseType::Naive);
  eulerSim->spawnParticle({400.0f, 100.0f}, {10.0f, 0.0f}, 5.0f, 1.0f);
  float initialY_euler = eulerSim->particles()[0].position.y;
  eulerSim->update();
  EXPECT_GT(eulerSim->particles()[0].position.y, initialY_euler);

  auto verletSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 1000,
      IntegrationType::Verlet, BroadphaseType::Naive);
  verletSim->spawnParticle({400.0f, 100.0f}, {10.0f, 0.0f}, 5.0f, 1.0f);
  float initialY_verlet = verletSim->particles()[0].position.y;
  verletSim->update();
  EXPECT_GT(verletSim->particles()[0].position.y, initialY_verlet);
}

TEST_F(SimulatorTest, SpawnedParticleVelocityNormBugFixed) {
  sim->spawnParticle({400.0f, 300.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);
  const auto& p = sim->particles()[0];
  float vn = p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y;
  EXPECT_GT(vn, 0.0f);
}
