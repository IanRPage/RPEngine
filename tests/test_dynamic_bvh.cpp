#include <gtest/gtest.h>

#include <broadphase/DynamicBVH.hpp>
#include <random>
#include <set>
#include <vector>

namespace {

AABB randomAABB(std::mt19937 &rng, float worldSize, float maxHalfExtent) {
  std::uniform_real_distribution<float> posDist(0.0f, worldSize);
  std::uniform_real_distribution<float> extDist(0.5f, maxHalfExtent);
  Vec3f center(posDist(rng), posDist(rng), posDist(rng));
  Vec3f half(extDist(rng), extDist(rng), extDist(rng));
  return AABB(center - half, center + half);
}

bool overlaps(const AABB &a, const AABB &b) {
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y &&
         a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

bool contains(const AABB &outer, const AABB &inner) {
  return outer.min.x <= inner.min.x && outer.max.x >= inner.max.x &&
         outer.min.y <= inner.min.y && outer.max.y >= inner.max.y &&
         outer.min.z <= inner.min.z && outer.max.z >= inner.max.z;
}

} // namespace

TEST(DynamicBVHTest, MatchesBruteForceOnRandomPoints) {
  for (uint32_t seed : {1u, 2u, 3u, 4u}) {
    std::mt19937 rng(seed);
    DynamicBVH tree;

    std::vector<std::pair<BodyHandle, AABB>> fattened;
    fattened.reserve(200);

    for (uint32_t i = 0; i < 200; i++) {
      AABB box = randomAABB(rng, 1000.0f, 5.0f);
      BodyHandle handle{i, 0};
      int32_t nodeId = tree.insert(handle, box);
      fattened.emplace_back(handle, tree.node(nodeId).fatAABB);
    }

    for (int q = 0; q < 20; q++) {
      AABB query = randomAABB(rng, 1000.0f, 20.0f);

      std::set<uint32_t> treeResult;
      tree.query(query, [&](int32_t, BodyHandle body) {
        treeResult.insert(body.index);
      });

      std::set<uint32_t> bruteResult;
      for (const auto &[handle, fatAABB] : fattened) {
        if (overlaps(query, fatAABB))
          bruteResult.insert(handle.index);
      }

      EXPECT_EQ(treeResult, bruteResult) << "seed=" << seed << " query=" << q;
    }
  }
}

TEST(DynamicBVHTest, SmallInMarginMoveCausesNoMutation) {
  DynamicBVH tree;
  AABB original(Vec3f(0.0f), Vec3f(1.0f));
  int32_t nodeId = tree.insert(BodyHandle{0, 0}, original);

  uint64_t mutationsBefore = tree.mutationCount();

  AABB shifted(original.min + Vec3f(0.02f, 0.0f, 0.0f),
               original.max + Vec3f(0.02f, 0.0f, 0.0f));
  bool moved = tree.moveProxy(nodeId, shifted, Vec3f(0.02f, 0.0f, 0.0f));

  EXPECT_FALSE(moved);
  EXPECT_EQ(tree.mutationCount(), mutationsBefore)
      << "moveProxy must be a true no-op (zero tree mutation) for an in-margin "
         "move";
}

TEST(DynamicBVHTest, LargeMoveTriggersReinsertion) {
  DynamicBVH tree;
  AABB original(Vec3f(0.0f), Vec3f(1.0f));
  int32_t nodeId = tree.insert(BodyHandle{0, 0}, original);

  AABB farAway(Vec3f(50.0f), Vec3f(51.0f));
  bool moved = tree.moveProxy(nodeId, farAway, Vec3f(1.0f, 0.0f, 0.0f));

  ASSERT_TRUE(moved);
  const AABB &newFat = tree.node(nodeId).fatAABB;
  EXPECT_TRUE(contains(newFat, farAway))
      << "the rebuilt fat AABB must actually contain the new real AABB (with "
         "margin)";
  EXPECT_FALSE(contains(newFat, original))
      << "the fat AABB must have actually moved, not just grown";
}

TEST(DynamicBVHTest, RemoveAndReinsertKeepsTreeValid) {
  DynamicBVH tree;
  std::mt19937 rng(7);

  std::vector<std::pair<BodyHandle, int32_t>> live;
  for (uint32_t i = 0; i < 50; i++) {
    BodyHandle handle{i, 0};
    AABB box = randomAABB(rng, 500.0f, 3.0f);
    int32_t nodeId = tree.insert(handle, box);
    live.emplace_back(handle, nodeId);
  }

  for (size_t i = 0; i < live.size(); i += 5) {
    tree.remove(live[i].second);
    AABB box = randomAABB(rng, 500.0f, 3.0f);
    live[i].second = tree.insert(live[i].first, box);
  }

  std::set<uint32_t> expected;
  for (const auto &[handle, nodeId] : live)
    expected.insert(handle.index);

  AABB covering(Vec3f(-1000.0f), Vec3f(1000.0f));
  std::set<uint32_t> actual;
  tree.query(covering,
             [&](int32_t, BodyHandle body) { actual.insert(body.index); });

  EXPECT_EQ(actual, expected);
}
