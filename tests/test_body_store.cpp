#include <gtest/gtest.h>

#include <core/BodyStore.hpp>

namespace {
AABB makeAABB(float x) {
  return AABB(Vec3f(x, 0.0f, 0.0f), Vec3f(x + 1.0f, 1.0f, 1.0f));
}
}  // namespace

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
  for (BodyHandle h : store.liveHandles()) { EXPECT_FALSE(h == b); }
}

TEST(BodyStoreTest, SetAABBUpdatesLiveBody) {
  BodyStore store;
  BodyHandle h = store.addBody(makeAABB(0.0f));
  store.setAABB(h, makeAABB(42.0f), Vec3f(1.0f, 0.0f, 0.0f));

  EXPECT_FLOAT_EQ(store.aabb(h).min.x, 42.0f);
  EXPECT_FLOAT_EQ(store.displacement(h).x, 1.0f);
}

TEST(BodyStoreTest, AABBOnlyAddBodyDefaultsToStaticDynamicsState) {
  BodyStore store;
  BodyHandle h = store.addBody(makeAABB(0.0f));

  EXPECT_FLOAT_EQ(store.invMass(h), 0.0f);
  EXPECT_FLOAT_EQ(store.linearVelocity(h).x, 0.0f);
  EXPECT_FALSE(store.constrainTo2D(h));
}

TEST(BodyStoreTest, BodyDescAddBodyInitializesDynamicsStateAndAABB) {
  BodyStore store;
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{2.0f}};
  desc.transform.position = Vec3f(3.0f, 4.0f, 0.0f);
  desc.invMass = 0.5f;
  desc.invInertiaBody = Mat3f(1.0f);
  desc.friction = 0.3f;
  desc.restitution = 0.7f;
  desc.constrainTo2D = true;

  BodyHandle h = store.addBody(desc);

  EXPECT_TRUE(store.isLive(h));
  EXPECT_FLOAT_EQ(store.position(h).x, 3.0f);
  EXPECT_FLOAT_EQ(store.position(h).y, 4.0f);
  EXPECT_FLOAT_EQ(store.prevPosition(h).x, 3.0f);
  EXPECT_FLOAT_EQ(store.invMass(h), 0.5f);
  EXPECT_FLOAT_EQ(store.friction(h), 0.3f);
  EXPECT_FLOAT_EQ(store.restitution(h), 0.7f);
  EXPECT_TRUE(store.constrainTo2D(h));

  EXPECT_FLOAT_EQ(store.aabb(h).min.x, 1.0f);
  EXPECT_FLOAT_EQ(store.aabb(h).max.x, 5.0f);
}

TEST(BodyStoreTest, MutableAccessorsWriteThrough) {
  BodyStore store;
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{1.0f}};
  BodyHandle h = store.addBody(desc);

  store.position(h) = Vec3f(1.0f, 2.0f, 3.0f);
  store.linearVelocity(h) = Vec3f(0.0f, -5.0f, 0.0f);
  store.orientation(h) = Quatf(0.0f, 1.0f, 0.0f, 0.0f);

  EXPECT_FLOAT_EQ(store.position(h).y, 2.0f);
  EXPECT_FLOAT_EQ(store.linearVelocity(h).y, -5.0f);
  EXPECT_FLOAT_EQ(store.transform(h).position.z, 3.0f);
  EXPECT_FLOAT_EQ(store.transform(h).orientation.x, 1.0f);
}

TEST(BodyStoreTest, RemoveThenReAddDoesNotInheritStaleDynamicsState) {
  BodyStore store;
  BodyDesc fastDesc;
  fastDesc.shape = ShapeVariant{SphereShape{1.0f}};
  fastDesc.invMass = 1.0f;
  BodyHandle fast = store.addBody(fastDesc);
  store.linearVelocity(fast) = Vec3f(50.0f, -25.0f, 0.0f);
  store.angularVelocity(fast) = Vec3f(0.0f, 0.0f, 10.0f);
  store.removeBody(fast);

  BodyDesc freshDesc;
  freshDesc.shape = ShapeVariant{SphereShape{1.0f}};
  freshDesc.invMass = 1.0f;
  BodyHandle fresh = store.addBody(freshDesc);

  ASSERT_EQ(fresh.index, fast.index);  // same slot, reused from free list
  EXPECT_FLOAT_EQ(store.linearVelocity(fresh).x, 0.0f);
  EXPECT_FLOAT_EQ(store.linearVelocity(fresh).y, 0.0f);
  EXPECT_FLOAT_EQ(store.angularVelocity(fresh).z, 0.0f);
}
