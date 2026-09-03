#include <gtest/gtest.h>

#include <broadphase/DynamicBVHBroadphase.hpp>
#include <broadphase/GridBroadphase.hpp>
#include <broadphase/NaiveBroadphase.hpp>
#include <core/BodyStore.hpp>
#include <random>
#include <set>
#include <utility>

namespace {

using IndexPair = std::pair<uint32_t, uint32_t>;

IndexPair canonical(BodyHandle a, BodyHandle b) {
  return (a.index < b.index) ? IndexPair{a.index, b.index}
                             : IndexPair{b.index, a.index};
}

std::set<IndexPair>
toIndexSet(std::span<const std::pair<BodyHandle, BodyHandle>> pairs) {
  std::set<IndexPair> result;
  for (const auto &[a, b] : pairs)
    result.insert(canonical(a, b));
  return result;
}

bool overlaps(const AABB &a, const AABB &b) {
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y &&
         a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

} // namespace

TEST(BroadphaseTest, AllImplementationsAgreeWithBruteForceOverlap) {
  BodyStore store;
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> posDist(0.0f, 20.0f);
  std::uniform_real_distribution<float> extDist(1.0f, 4.0f);

  std::vector<BodyHandle> handles;
  for (int i = 0; i < 60; i++) {
    Vec3f center(posDist(rng), posDist(rng), posDist(rng));
    Vec3f half(extDist(rng), extDist(rng), extDist(rng));
    handles.push_back(store.addBody(AABB(center - half, center + half)));
  }

  std::set<IndexPair> realOverlaps;
  for (size_t i = 0; i < handles.size(); i++) {
    for (size_t j = i + 1; j < handles.size(); j++) {
      if (overlaps(store.aabb(handles[i]), store.aabb(handles[j]))) {
        realOverlaps.insert(canonical(handles[i], handles[j]));
      }
    }
  }
  ASSERT_GT(realOverlaps.size(), 0u)
      << "test fixture produced no overlaps to exercise";

  NaiveBroadphase naive;
  std::set<IndexPair> naivePairs = toIndexSet(naive.computePairs(store));

  std::set<IndexPair> allPairs;
  for (size_t i = 0; i < handles.size(); i++) {
    for (size_t j = i + 1; j < handles.size(); j++) {
      allPairs.insert(canonical(handles[i], handles[j]));
    }
  }
  EXPECT_EQ(naivePairs, allPairs)
      << "NaiveBroadphase must emit every unique pair unconditionally";

  GridBroadphase grid;
  std::set<IndexPair> gridPairs = toIndexSet(grid.computePairs(store));
  for (const IndexPair &p : realOverlaps) {
    EXPECT_TRUE(gridPairs.count(p) != 0)
        << "GridBroadphase missed a real overlap";
  }

  DynamicBVHBroadphase bvh;
  std::set<IndexPair> bvhPairs = toIndexSet(bvh.computePairs(store));
  for (const IndexPair &p : realOverlaps) {
    EXPECT_TRUE(bvhPairs.count(p) != 0)
        << "DynamicBVHBroadphase missed a real overlap";
  }
}

TEST(BroadphaseTest, GridDoesNotMissSmallBodyOverlappingCenterDistantLargeBody) {
  BodyStore store;

  BodyHandle small = store.addBody(
      AABB(Vec3f(-0.5f, -0.5f, -0.5f), Vec3f(0.5f, 0.5f, 0.5f)));
  // center far from `small`'s center-cell, but its so large it still reaches
  // back to overlap `small`
  BodyHandle large = store.addBody(
      AABB(Vec3f(-1.0f, -15.0f, -15.0f), Vec3f(29.0f, 15.0f, 15.0f)));

  ASSERT_TRUE(overlaps(store.aabb(small), store.aabb(large)));

  GridBroadphase grid(2.0f);  // small cells so the two land far apart
  std::set<IndexPair> pairs = toIndexSet(grid.computePairs(store));

  EXPECT_TRUE(pairs.count(canonical(small, large)) != 0)
      << "GridBroadphase missed a small body overlapping a center-distant "
         "large body";
}

TEST(BroadphaseTest, DynamicBVHHandlesSameIndexBodyReplacement) {
  BodyStore store;

  BodyHandle stationary =
      store.addBody(AABB(Vec3f(0.0f), Vec3f(2.0f, 2.0f, 2.0f)));
  BodyHandle original = store.addBody(
      AABB(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(3.0f, 3.0f, 3.0f)));

  DynamicBVHBroadphase bvh;
  auto initial = toIndexSet(bvh.computePairs(store));
  ASSERT_TRUE(initial.count(canonical(stationary, original)) != 0);

  store.removeBody(original);
  BodyHandle replacement = store.addBody(
      AABB(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(3.0f, 3.0f, 3.0f)));
  ASSERT_EQ(replacement.index, original.index);
  ASSERT_NE(replacement.generation, original.generation);

  auto after = toIndexSet(bvh.computePairs(store));
  EXPECT_TRUE(after.count(canonical(stationary, replacement)) != 0)
      << "DynamicBVHBroadphase dropped a new overlap on a reused node id";
}
