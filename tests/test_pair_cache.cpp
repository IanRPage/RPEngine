#include <gtest/gtest.h>

#include <broadphase/PairCache.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

TEST(PairCacheTest, DetectsNewAndRemovedOverlaps) {
  DynamicBVH tree;
  PairCache cache;

  BodyHandle a{0, 0}, b{1, 0};
  AABB boxA(Vec3f(0.0f), Vec3f(1.0f));
  AABB boxBFar(Vec3f(100.0f), Vec3f(101.0f));

  int32_t nodeA = tree.insert(a, boxA);
  int32_t nodeB = tree.insert(b, boxBFar);

  cache.update(tree, std::vector<int32_t>{nodeA, nodeB});
  EXPECT_TRUE(cache.pairs().empty());

  // displacement is a per-step predictive-fattening delta, not the distance
  // travelled to reach the target AABB. only care if target lands outside
  // current fat AABB
  AABB boxBNear(Vec3f(0.5f), Vec3f(1.5f));
  ASSERT_TRUE(tree.moveProxy(nodeB, boxBNear, Vec3f(0.0f)));
  cache.update(tree, std::vector<int32_t>{nodeB});

  ASSERT_EQ(cache.pairs().size(), 1u);
  auto pair = cache.pairs()[0];
  EXPECT_TRUE((pair.first == a && pair.second == b) ||
              (pair.first == b && pair.second == a));

  ASSERT_TRUE(tree.moveProxy(nodeB, boxBFar, Vec3f(0.0f)));
  cache.update(tree, std::vector<int32_t>{nodeB});

  EXPECT_TRUE(cache.pairs().empty());
}

TEST(PairCacheTest, DoesNotDuplicatePairs) {
  DynamicBVH tree;
  PairCache cache;

  BodyHandle a{0, 0}, b{1, 0};
  AABB boxA(Vec3f(0.0f), Vec3f(2.0f));
  AABB boxB(Vec3f(1.0f), Vec3f(3.0f));

  int32_t nodeA = tree.insert(a, boxA);
  int32_t nodeB = tree.insert(b, boxB);

  cache.update(tree, std::vector<int32_t>{nodeA, nodeB});
  ASSERT_EQ(cache.pairs().size(), 1u);

  AABB boxB2(Vec3f(1.5f, 1.0f, 1.0f), Vec3f(3.5f, 3.0f, 3.0f));
  ASSERT_TRUE(tree.moveProxy(nodeB, boxB2, Vec3f(0.0f)));
  cache.update(tree, std::vector<int32_t>{nodeB});
  EXPECT_EQ(cache.pairs().size(), 1u);

  AABB boxB3(Vec3f(1.8f, 1.0f, 1.0f), Vec3f(3.8f, 3.0f, 3.0f));
  ASSERT_TRUE(tree.moveProxy(nodeB, boxB3, Vec3f(0.0f)));
  cache.update(tree, std::vector<int32_t>{nodeB});
  EXPECT_EQ(cache.pairs().size(), 1u);
}

TEST(PairCacheTest, PerformanceAt100kBodiesBenchmark) {
  constexpr uint32_t kBodyCount = 100000;

  DynamicBVH tree;
  PairCache cache;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> posDist(0.0f, 5000.0f);
  std::uniform_real_distribution<float> jitterDist(-0.01f, 0.01f);

  std::vector<int32_t> nodeIds;
  std::vector<AABB> realAABBs;
  nodeIds.reserve(kBodyCount);
  realAABBs.reserve(kBodyCount);

  for (uint32_t i = 0; i < kBodyCount; i++) {
    Vec3f center(posDist(rng), posDist(rng), posDist(rng));
    AABB box(center - Vec3f(0.5f), center + Vec3f(0.5f));
    nodeIds.push_back(tree.insert(BodyHandle{i, 0}, box));
    realAABBs.push_back(box);
  }

  cache.update(tree, nodeIds); // prime cache w initial layout

  std::vector<int32_t> moved;
  for (uint32_t i = 0; i < kBodyCount; i++) {
    Vec3f disp = (i % 100 == 0)
                     ? Vec3f(posDist(rng) * 0.01f, 0.0f, 0.0f)
                     : Vec3f(jitterDist(rng), jitterDist(rng), jitterDist(rng));
    realAABBs[i] = AABB(realAABBs[i].min + disp, realAABBs[i].max + disp);
    if (tree.moveProxy(nodeIds[i], realAABBs[i], disp))
      moved.push_back(nodeIds[i]);
  }

  auto start = std::chrono::steady_clock::now();
  cache.update(tree, moved);
  auto end = std::chrono::steady_clock::now();

  double ms = std::chrono::duration<double, std::milli>(end - start).count();
  std::cout << "[PairCache benchmark] " << kBodyCount << " bodies, "
            << moved.size() << " moved, pairs=" << cache.pairs().size()
            << ", update() took " << ms << " ms\n";
  SUCCEED();
}
