#include <gtest/gtest.h>

#include <core/BodyStore.hpp>

namespace {
AABB makeAABB(float x) {
  return AABB(Vec3f(x, 0.0f, 0.0f), Vec3f(x + 1.0f, 1.0f, 1.0f));
}
} // namespace

TEST(BodyStoreTest, AddBodyIsLiveAndReportsAABB) {
  BodyStore store;
  BodyHandle h = store.addBody(makeAABB(5.0f));

  EXPECT_TRUE(h.valid());
  EXPECT_TRUE(store.isLive(h));
  EXPECT_EQ(store.size(), 1u);
  EXPECT_FLOAT_EQ(store.aabb(h).min.x, 5.0f);
}

TEST(BodyStoreTest, RemoveBodyMakesHandleStale) {
  BodyStore store;
  BodyHandle h = store.addBody(makeAABB(0.0f));
  store.removeBody(h);

  EXPECT_FALSE(store.isLive(h));
  EXPECT_EQ(store.size(), 0u);
}

TEST(BodyStoreTest, RemoveThenReAddReusesSlotWithBumpedGeneration) {
  BodyStore store;
  BodyHandle first = store.addBody(makeAABB(0.0f));
  store.removeBody(first);

  BodyHandle second = store.addBody(makeAABB(10.0f));

  EXPECT_EQ(second.index, first.index);
  EXPECT_NE(second.generation, first.generation);

  // stale first handle can't resolve to new body's data
  EXPECT_FALSE(store.isLive(first));
  EXPECT_TRUE(store.isLive(second));
  EXPECT_FLOAT_EQ(store.aabb(second).min.x, 10.0f);
}

TEST(BodyStoreTest, RemoveMiddleBodyKeepsOthersLiveViaSwapErase) {
  BodyStore store;
  BodyHandle a = store.addBody(makeAABB(0.0f));
  BodyHandle b = store.addBody(makeAABB(1.0f));
  BodyHandle c = store.addBody(makeAABB(2.0f));

  store.removeBody(b);

  EXPECT_TRUE(store.isLive(a));
  EXPECT_FALSE(store.isLive(b));
  EXPECT_TRUE(store.isLive(c));
  EXPECT_EQ(store.size(), 2u);

  // liveHandles() can't list removed handle
  for (BodyHandle h : store.liveHandles()) {
    EXPECT_FALSE(h == b);
  }
}

TEST(BodyStoreTest, SetAABBUpdatesLiveBody) {
  BodyStore store;
  BodyHandle h = store.addBody(makeAABB(0.0f));
  store.setAABB(h, makeAABB(42.0f), Vec3f(1.0f, 0.0f, 0.0f));

  EXPECT_FLOAT_EQ(store.aabb(h).min.x, 42.0f);
  EXPECT_FLOAT_EQ(store.displacement(h).x, 1.0f);
}
