#include <Particle.hpp>
#include <dsa/QuadTree.hpp>
#include <gtest/gtest.h>

TEST(QuadTreeTest, InsertAndQuerySingleParticle) {
  QuadTree<Particle> qt(AABBf({0.0f, 0.0f}, {100.0f, 100.0f}), 4);
  Particle p({50.0f, 50.0f}, {0.0f, 0.0f});
  EXPECT_TRUE(qt.insert(&p));

  std::vector<Particle*> results;
  qt.query(results, AABBf({40.0f, 40.0f}, {20.0f, 20.0f}));
  EXPECT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0], &p);
}

TEST(QuadTreeTest, InsertOutsideBoundaryFails) {
  QuadTree<Particle> qt(AABBf({0.0f, 0.0f}, {100.0f, 100.0f}), 4);
  Particle p({150.0f, 150.0f}, {0.0f, 0.0f});
  EXPECT_FALSE(qt.insert(&p));
}

TEST(QuadTreeTest, QueryReturnsOnlyInRange) {
  QuadTree<Particle> qt(AABBf({0.0f, 0.0f}, {200.0f, 200.0f}), 4);
  Particle p1({10.0f, 10.0f}, {0.0f, 0.0f});
  Particle p2({190.0f, 190.0f}, {0.0f, 0.0f});
  qt.insert(&p1);
  qt.insert(&p2);

  std::vector<Particle*> results;
  qt.query(results, AABBf({0.0f, 0.0f}, {50.0f, 50.0f}));
  EXPECT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0], &p1);
}

TEST(QuadTreeTest, SubdivisionWithManyParticles) {
  QuadTree<Particle> qt(AABBf({0.0f, 0.0f}, {100.0f, 100.0f}), 2);
  std::vector<Particle> particles;
  particles.reserve(10);
  for (int i = 0; i < 10; i++) {
    particles.emplace_back(Vec2f(10.0f * i + 5.0f, 50.0f), Vec2f(0.0f, 0.0f));
  }
  for (auto& p : particles) {
    qt.insert(&p);
  }

  std::vector<Particle*> results;
  qt.query(results, AABBf({0.0f, 0.0f}, {100.0f, 100.0f}));
  EXPECT_EQ(results.size(), 10u);
}

TEST(QuadTreeTest, QueryEmptyTreeReturnsNothing) {
  QuadTree<Particle> qt(AABBf({0.0f, 0.0f}, {100.0f, 100.0f}), 4);
  std::vector<Particle*> results;
  qt.query(results, AABBf({0.0f, 0.0f}, {100.0f, 100.0f}));
  EXPECT_EQ(results.size(), 0u);
}
