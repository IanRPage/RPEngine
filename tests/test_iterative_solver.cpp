#include <Simulator.hpp>
#include <cmath>
#include <gtest/gtest.h>

class IterativeSolverTest : public ::testing::Test {
protected:
  void SetUp() override {
    sim = std::make_unique<Simulator>(
        Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 10000,
        IntegrationType::Verlet, BroadphaseType::Naive);
  }
  std::unique_ptr<Simulator> sim;
};

TEST_F(IterativeSolverTest, MultipleOverlappingBodiesConverge) {
  for (int i = 0; i < 10; i++) {
    sim->spawnParticle({400.0f + i * 5.0f, 300.0f}, {0.0f, 0.0f}, 10.0f, 1.0f);
  }

  for (int frame = 0; frame < 200; frame++) {
    sim->update();
  }

  const auto &particles = sim->particles();
  for (size_t i = 0; i < particles.size(); i++) {
    for (size_t j = i + 1; j < particles.size(); j++) {
      float dx = particles[j].position.x - particles[i].position.x;
      float dy = particles[j].position.y - particles[i].position.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      float sumR = particles[i].radius + particles[j].radius;
      EXPECT_GE(dist, sumR - 1.0f) << "Particles " << i << " and " << j
                                   << " are overlapping after convergence";
    }
  }
}

TEST_F(IterativeSolverTest, StackedParticlesStable) {
  float r = 5.0f;
  for (int i = 0; i < 5; i++) {
    sim->spawnParticle({400.0f, 590.0f - i * 2.0f * r}, {0.0f, 0.0f}, r, 1.0f);
  }

  for (int frame = 0; frame < 500; frame++) {
    sim->update();
  }

  const auto &particles = sim->particles();
  for (const auto &p : particles) {
    EXPECT_FALSE(std::isnan(p.position.x));
    EXPECT_FALSE(std::isnan(p.position.y));
    EXPECT_GE(p.position.y, p.radius);
    EXPECT_LE(p.position.y, 600.0f - p.radius + 1.0f);
  }

  for (size_t i = 0; i < particles.size(); i++) {
    for (size_t j = i + 1; j < particles.size(); j++) {
      float dx = particles[j].position.x - particles[i].position.x;
      float dy = particles[j].position.y - particles[i].position.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      float sumR = particles[i].radius + particles[j].radius;
      EXPECT_GE(dist, sumR - 1.5f);
    }
  }
}

TEST_F(IterativeSolverTest, NoEnergyExplosion) {
  for (int i = 0; i < 20; i++) {
    sim->spawnParticle({400.0f + (i % 5) * 8.0f, 550.0f - (i / 5) * 8.0f},
                       {0.0f, 0.0f}, 5.0f, 1.0f);
  }

  float maxSpeed = 0.0f;
  for (int frame = 0; frame < 500; frame++) {
    sim->update();
    for (const auto &p : sim->particles()) {
      Vec2f v = p.position - p.prevPosition;
      float speed = std::sqrt(v.x * v.x + v.y * v.y);
      maxSpeed = std::max(maxSpeed, speed);
    }
  }

  EXPECT_LT(maxSpeed, 100.0f)
      << "Particle speeds should not explode; max speed was " << maxSpeed;
}

TEST_F(IterativeSolverTest, WorksWithAllBroadphases) {
  auto testBroadphase = [](BroadphaseType bp) {
    auto s = std::make_unique<Simulator>(Vec2f(400.0f, 400.0f), 10.0f, 9.81f,
                                         0.5f, 1.0f / 60.0f, 10000,
                                         IntegrationType::Verlet, bp);
    for (int i = 0; i < 8; i++) {
      s->spawnParticle({200.0f + i * 3.0f, 200.0f}, {0.0f, 0.0f}, 5.0f, 1.0f);
    }
    for (int frame = 0; frame < 100; frame++) {
      s->update();
    }
    for (const auto &p : s->particles()) {
      EXPECT_FALSE(std::isnan(p.position.x));
      EXPECT_FALSE(std::isnan(p.position.y));
      EXPECT_GE(p.position.x, p.radius - 0.5f);
      EXPECT_LE(p.position.x, 400.0f - p.radius + 0.5f);
    }
  };

  testBroadphase(BroadphaseType::Naive);
  testBroadphase(BroadphaseType::SpatialGrid);
}

TEST_F(IterativeSolverTest, EulerIterativeSolverWorks) {
  auto eulerSim = std::make_unique<Simulator>(
      Vec2f(800.0f, 600.0f), 20.0f, 9.81f, 0.5f, 1.0f / 60.0f, 10000,
      IntegrationType::Euler, BroadphaseType::Naive);

  for (int i = 0; i < 10; i++) {
    eulerSim->spawnParticle({400.0f + i * 5.0f, 300.0f}, {0.0f, 0.0f}, 10.0f,
                            1.0f);
  }

  for (int frame = 0; frame < 200; frame++) {
    eulerSim->update();
  }

  const auto &particles = eulerSim->particles();
  for (size_t i = 0; i < particles.size(); i++) {
    for (size_t j = i + 1; j < particles.size(); j++) {
      float dx = particles[j].position.x - particles[i].position.x;
      float dy = particles[j].position.y - particles[i].position.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      float sumR = particles[i].radius + particles[j].radius;
      EXPECT_GE(dist, sumR - 1.5f);
    }
  }
}
