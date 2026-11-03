#include <gtest/gtest.h>

#include <core/World.hpp>
#include <sim/Simulator.hpp>

namespace {

constexpr float kDt = 1.0f / 60.0f;
const Quatf kIdentity(1.0f, 0.0f, 0.0f, 0.0f);

BodyHandle addGround(World& world) {
  return world.createStaticBody(BoxShape{Vec3f(10.0f, 0.5f, 10.0f)},
                                Transform{Vec3f(0.0f, -0.5f, 0.0f), kIdentity},
                                0.5f, 0.0f);
}

BodyHandle addBox(World& world, Vec3f pos) {
  return world.createDynamicBody(BoxShape{Vec3f(0.5f)},
                                 Transform{pos, kIdentity}, 1.0f, 0.5f, 0.0f);
}

void stepFor(World& world, float seconds) {
  int n = static_cast<int>(seconds / kDt);
  for (int i = 0; i < n; ++i) {
    world.snapshotPrevState();
    world.step(kDt);
  }
}

}  // namespace

TEST(SleepingTest, ConfigDefaults) {
  SolverConfig c;
  EXPECT_TRUE(c.enableSleeping);
  EXPECT_FLOAT_EQ(c.restitutionVelocityThreshold, 0.5f);
  EXPECT_FLOAT_EQ(c.linearSleepThreshold, 0.05f);
  EXPECT_FLOAT_EQ(c.angularSleepThreshold, 0.05f);
  EXPECT_FLOAT_EQ(c.timeToSleep, 0.5f);
}

TEST(SleepingTest, BodyStoreSleepWakeApi) {
  World world;
  BodyHandle ground = addGround(world);
  BodyHandle box = addBox(world, Vec3f(0.0f, 5.0f, 0.0f));
  BodyStore& b = world.bodies();
  EXPECT_FALSE(b.isSleeping(box));
  EXPECT_FLOAT_EQ(b.sleepTimer(box), 0.0f);

  b.putToSleep(ground);
  EXPECT_FALSE(b.isSleeping(ground));

  b.linearVelocity(box) = Vec3f(1.0f);
  b.putToSleep(box);
  EXPECT_TRUE(b.isSleeping(box));
  EXPECT_EQ(b.linearVelocity(box), Vec3f(0.0f));
  EXPECT_EQ(b.displacement(box), Vec3f(0.0f));
  EXPECT_EQ(world.sleepingBodyCount(), 1u);

  b.wake(box);
  EXPECT_FALSE(b.isSleeping(box));
  EXPECT_EQ(world.sleepingBodyCount(), 0u);
}

TEST(SleepingTest, RestingBoxFallsAsleepAndStaysPut) {
  World world;
  BodyHandle ground = addGround(world);
  BodyHandle box = addBox(world, Vec3f(0.0f, 0.6f, 0.0f));
  stepFor(world, 3.0f);
  ASSERT_TRUE(world.bodies().isSleeping(box));
  EXPECT_FALSE(world.bodies().isSleeping(ground));
  EXPECT_EQ(world.sleepingBodyCount(), 1u);

  Vec3f pos = world.bodies().position(box);
  Quatf q = world.bodies().orientation(box);
  stepFor(world, 2.0f);
  EXPECT_EQ(world.bodies().position(box), pos);
  EXPECT_EQ(world.bodies().orientation(box), q);
  EXPECT_EQ(world.bodies().prevPosition(box), pos);
  EXPECT_EQ(world.bodies().prevOrientation(box), q);
  EXPECT_EQ(world.bodies().linearVelocity(box), Vec3f(0.0f));
}

TEST(SleepingTest, SleepingSkipsNarrowphase) {
  World world;
  int calls = 0;
  world.setNarrowphaseFns(
      [&](const ShapeVariant& a, const Transform& ta, const ShapeVariant& b,
          const Transform& tb) {
        calls++;
        return gjkOverlap(a, ta, b, tb);
      },
      nullptr);
  addGround(world);
  BodyHandle box = addBox(world, Vec3f(0.0f, 0.5f, 0.0f));
  stepFor(world, 3.0f);
  ASSERT_TRUE(world.bodies().isSleeping(box));
  calls = 0;
  stepFor(world, 1.0f);
  EXPECT_EQ(calls, 0);
}

TEST(SleepingTest, StackSleepsAsIslandAndWakesTogether) {
  World world;
  addGround(world);
  BodyHandle a = addBox(world, Vec3f(0.0f, 0.5f, 0.0f));
  BodyHandle b = addBox(world, Vec3f(0.0f, 1.5f, 0.0f));
  BodyHandle c = addBox(world, Vec3f(0.0f, 2.5f, 0.0f));
  BodyStore& s = world.bodies();
  bool sawPartial = false;
  for (int i = 0; i < 300; ++i) {
    world.step(kDt);
    int n = s.isSleeping(a) + s.isSleeping(b) + s.isSleeping(c);
    if (n != 0 && n != 3) { sawPartial = true; }
  }
  EXPECT_FALSE(sawPartial);
  ASSERT_EQ(world.sleepingBodyCount(), 3u);
  float topY = s.position(c).y;

  s.wake(b);
  world.step(kDt);
  EXPECT_FALSE(s.isSleeping(a));
  EXPECT_FALSE(s.isSleeping(b));
  EXPECT_FALSE(s.isSleeping(c));
  EXPECT_NEAR(s.position(c).y, topY, 1e-3f);

  stepFor(world, 3.0f);
  EXPECT_EQ(world.sleepingBodyCount(), 3u);
  EXPECT_NEAR(s.position(c).y, topY, 2e-2f);
}

TEST(SleepingTest, SeparateIslandsOnSameStaticGroundAreIndependent) {
  World world;
  addGround(world);
  BodyHandle left = addBox(world, Vec3f(-5.0f, 0.5f, 0.0f));
  BodyHandle right = addBox(world, Vec3f(5.0f, 0.5f, 0.0f));
  world.bodies().linearVelocity(right) = Vec3f(0.0f, 4.0f, 0.0f);

  stepFor(world, 0.6f);
  EXPECT_TRUE(world.bodies().isSleeping(left));
  EXPECT_FALSE(world.bodies().isSleeping(right));

  stepFor(world, 3.0f);
  ASSERT_TRUE(world.bodies().isSleeping(left));
  ASSERT_TRUE(world.bodies().isSleeping(right));

  // static ground does not bridge islands: waking left leaves right asleep
  world.bodies().wake(left);
  world.step(kDt);
  EXPECT_FALSE(world.bodies().isSleeping(left));
  EXPECT_TRUE(world.bodies().isSleeping(right));
}

TEST(SleepingTest, AwakeBodyHittingSleepingIslandWakesIt) {
  World world;
  addGround(world);
  BodyHandle a = addBox(world, Vec3f(0.0f, 0.5f, 0.0f));
  BodyHandle b = addBox(world, Vec3f(0.0f, 1.5f, 0.0f));
  stepFor(world, 3.0f);
  ASSERT_EQ(world.sleepingBodyCount(), 2u);

  BodyHandle dropped = addBox(world, Vec3f(0.0f, 4.0f, 0.0f));
  bool woke = false;
  for (int i = 0; i < 120 && !woke; ++i) {
    world.step(kDt);
    woke = !world.bodies().isSleeping(a) && !world.bodies().isSleeping(b);
  }
  EXPECT_TRUE(woke);
  EXPECT_FALSE(world.bodies().isSleeping(dropped));
  stepFor(world, 4.0f);
  EXPECT_EQ(world.sleepingBodyCount(), 3u);
}

TEST(SleepingTest, RemovingOrMovingSupportWakesSleepingIsland) {
  World world;
  BodyHandle ground = addGround(world);
  BodyHandle a = addBox(world, Vec3f(0.0f, 0.5f, 0.0f));
  BodyHandle b = addBox(world, Vec3f(0.0f, 1.5f, 0.0f));
  stepFor(world, 3.0f);
  ASSERT_EQ(world.sleepingBodyCount(), 2u);
  float topY = world.bodies().position(b).y;

  world.bodies().removeBody(a);
  EXPECT_EQ(world.sleepingBodyCount(), 1u);

  world.step(kDt);
  EXPECT_FALSE(world.bodies().isSleeping(b));
  stepFor(world, 3.0f);
  EXPECT_LT(world.bodies().position(b).y, topY - 0.5f);
  ASSERT_TRUE(world.bodies().isSleeping(b));

  // moving static support away wakes the resting sleeper on the next step
  float settledY = world.bodies().position(b).y;
  world.bodies().position(ground).y -= 5.0f;
  world.step(kDt);
  EXPECT_FALSE(world.bodies().isSleeping(b));
  stepFor(world, 0.5f);
  EXPECT_LT(world.bodies().position(b).y, settledY - 0.5f);
}

TEST(SleepingTest, DisablingSleepAndWakeAll) {
  World world;
  addGround(world);
  addBox(world, Vec3f(0.0f, 0.5f, 0.0f));
  addBox(world, Vec3f(3.0f, 0.5f, 0.0f));
  stepFor(world, 3.0f);
  ASSERT_EQ(world.sleepingBodyCount(), 2u);

  world.wakeAll();
  EXPECT_EQ(world.sleepingBodyCount(), 0u);

  stepFor(world, 3.0f);
  ASSERT_EQ(world.sleepingBodyCount(), 2u);

  world.config().enableSleeping = false;
  world.step(kDt);
  EXPECT_EQ(world.sleepingBodyCount(), 0u);
  stepFor(world, 3.0f);
  EXPECT_EQ(world.sleepingBodyCount(), 0u);
}

TEST(SleepingTest, BoxPileWithRestitutionSettlesAsleep) {
  Simulator sim;
  World& world = sim.world();
  world.createStaticBody(BoxShape{Vec3f(13.5f, 0.5f, 13.5f)},
                         Transform{Vec3f(0.0f, -0.5f, 0.0f), kIdentity}, 0.5f,
                         0.3f);
  std::vector<BodyHandle> boxes;
  for (int i = 0; i < 40; ++i) {
    float x = static_cast<float>((i * 37) % 400 - 200) / 100.0f;
    float z = static_cast<float>((i * 91) % 400 - 200) / 100.0f;
    Transform t{Vec3f(x, 1.0f + i * 0.6f, z), kIdentity};
    boxes.push_back(world.createDynamicBody(BoxShape{Vec3f(0.5f)}, t, 1.0f,
                                            0.5f, 0.3f));
  }
  for (int i = 0; i < 60 * 15; ++i) { sim.advance(kDt); }
  size_t onPlatform = 0;
  for (BodyHandle h : boxes) {
    if (world.bodies().position(h).y > -0.5f) { onPlatform++; }
  }
  EXPECT_GT(onPlatform, 20u);
  EXPECT_EQ(world.sleepingBodyCount(), onPlatform);
}
