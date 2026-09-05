#include <Particle.hpp>
#include <dsa/SpatialGrid.hpp>
#include <gtest/gtest.h>

TEST(SpatialGridTest, ConfigureSetsGridDimensions) {
  SpatialGrid grid;
  grid.configure(10.0f, {100.0f, 100.0f});
  EXPECT_EQ(grid.cols, 10);
  EXPECT_EQ(grid.rows, 10);
  EXPECT_EQ(grid.nCells, 100);
  EXPECT_FLOAT_EQ(grid.invCellSize, 0.1f);
}

TEST(SpatialGridTest, BuildAndQueryFindsNeighbors) {
  SpatialGrid grid;
  grid.configure(20.0f, {100.0f, 100.0f});

  std::vector<Particle> particles;
  particles.emplace_back(Vec2f(10.0f, 10.0f), Vec2f(0.0f, 0.0f), 1.0f / 60.0f,
                         5.0f, 1.0f);
  particles.emplace_back(Vec2f(15.0f, 10.0f), Vec2f(0.0f, 0.0f), 1.0f / 60.0f,
                         5.0f, 1.0f);
  particles.emplace_back(Vec2f(90.0f, 90.0f), Vec2f(0.0f, 0.0f), 1.0f / 60.0f,
                         5.0f, 1.0f);

  grid.resize(particles.size());
  grid.build(particles);

  std::vector<int> neighbors;
  grid.queryDoSomething(0, 5.0f, particles[0].position,
                        [&](int idx) { neighbors.push_back(idx); });

  EXPECT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors[0], 1);
}

TEST(SpatialGridTest, ResizeHandlesDifferentSizes) {
  SpatialGrid grid;
  grid.configure(10.0f, {100.0f, 100.0f});
  grid.resize(50);
  EXPECT_EQ(grid.headSize, static_cast<size_t>(grid.nCells));
  EXPECT_EQ(grid.nextSize, 50u);

  grid.resize(100);
  EXPECT_EQ(grid.nextSize, 100u);
}

TEST(SpatialGridTest, QueryPrunesDuplicates) {
  SpatialGrid grid;
  grid.configure(50.0f, {100.0f, 100.0f});

  std::vector<Particle> particles;
  particles.emplace_back(Vec2f(25.0f, 25.0f), Vec2f(0.0f, 0.0f), 1.0f / 60.0f,
                         5.0f, 1.0f);
  particles.emplace_back(Vec2f(30.0f, 25.0f), Vec2f(0.0f, 0.0f), 1.0f / 60.0f,
                         5.0f, 1.0f);

  grid.resize(particles.size());
  grid.build(particles);

  std::vector<int> fromP0, fromP1;
  grid.queryDoSomething(0, 5.0f, particles[0].position,
                        [&](int idx) { fromP0.push_back(idx); });
  grid.queryDoSomething(1, 5.0f, particles[1].position,
                        [&](int idx) { fromP1.push_back(idx); });

  EXPECT_EQ(fromP0.size(), 1u);
  EXPECT_EQ(fromP0[0], 1);
  EXPECT_EQ(fromP1.size(), 0u);
}
