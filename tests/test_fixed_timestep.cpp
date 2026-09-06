#include <gtest/gtest.h>

#include <sim/Simulator.hpp>

namespace {
BodyHandle spawnFreeBody(Simulator& sim, Vec3f position, Vec3f linearVelocity) {
  Transform t;
  t.position = position;
  BodyHandle h = sim.world().createDynamicBody(
      ShapeVariant{SphereShape{0.5f}}, t, /*mass=*/1.0f, /*friction=*/0.5f,
      /*restitution=*/0.0f, /*constrainTo2D=*/false);
  sim.world().bodies().linearVelocity(h) = linearVelocity;
  return h;
}
}  // namespace

TEST(FixedTimestepTest, StepCountMatchesFloorOfElapsedOverFixedDt) {
  Simulator sim;  // fixedDt = 1/60 by default
  // Derived from fixedDt (rather than a literal like 0.05f) so it isn't at
  // the mercy of 0.05f and 1/60 rounding to float32 in slightly different
  // directions -- 3.5x guarantees 3 whole steps regardless of that.
  sim.advance(3.5f * sim.config().fixedDt);
  EXPECT_EQ(sim.stepsTakenLastAdvance(), 3);
}

TEST(FixedTimestepTest, AlphaStaysInZeroOneRange) {
  Simulator sim;
  // Deltas chosen so no single advance() call ever needs more than
  // maxStepsPerFrame (5) substeps -- MaxStepsPerFrameClampsRunawayAccumulator
  // below is the dedicated test for what happens once that clamp triggers,
  // so this test stays in the non-clamping regime on purpose.
  const float deltas[] = {0.013f, 0.02f, 0.004f};
  for (int i = 0; i < 200; i++) {
    sim.advance(deltas[i % 3]);
    float alpha = sim.interpolationAlpha();
    EXPECT_GE(alpha, 0.0f);
    EXPECT_LT(alpha, 1.0f);
  }
}

TEST(FixedTimestepTest, ConstantVelocityBodyInterpolatesExactly) {
  Simulator sim;
  sim.world().setGravity(Vec3f(0.0f));
  const Vec3f spawnPos(0.0f, 0.0f, 0.0f);
  const Vec3f velocity(2.0f, 0.0f, 0.0f);
  BodyHandle h = spawnFreeBody(sim, spawnPos, velocity);

  const float fixedDt = sim.config().fixedDt;
  // 1.5 * fixedDt -> exactly one substep taken, alpha == 0.5. Kept to a
  // single substep so prevPos/currentPos span exactly one fixedDt of
  // motion, matching the hand-computed formula below term for term.
  sim.advance(1.5f * fixedDt);
  ASSERT_EQ(sim.stepsTakenLastAdvance(), 1);

  float alpha = sim.interpolationAlpha();
  Vec3f prevPos = sim.world().bodies().prevPosition(h);
  Vec3f currPos = sim.world().bodies().position(h);
  Vec3f renderPos = glm::mix(prevPos, currPos, alpha);

  Vec3f expected = prevPos + velocity * (alpha * fixedDt);
  EXPECT_NEAR(renderPos.x, expected.x, 1e-5f);
  EXPECT_NEAR(renderPos.y, expected.y, 1e-5f);
  EXPECT_NEAR(renderPos.z, expected.z, 1e-5f);
}

TEST(FixedTimestepTest, MaxStepsPerFrameClampsRunawayAccumulator) {
  Simulator sim;  // maxStepsPerFrame = 5 by default
  const float fixedDt = sim.config().fixedDt;
  const int maxSteps = sim.config().maxStepsPerFrame;

  sim.advance(10.0f);  // simulates a multi-second stall

  EXPECT_EQ(sim.stepsTakenLastAdvance(), maxSteps);

  // Leftover time must be retained, not discarded -- recovered from alpha
  // since alpha == accumulator / fixedDt.
  float recoveredAccumulator = sim.interpolationAlpha() * fixedDt;
  float expectedLeftover = 10.0f - static_cast<float>(maxSteps) * fixedDt;
  EXPECT_NEAR(recoveredAccumulator, expectedLeftover, 1e-3f);
}

TEST(FixedTimestepTest, PrevPositionCapturedBeforeFirstOfMultipleSubsteps) {
  Simulator sim;
  sim.world().setGravity(Vec3f(0.0f, -9.81f, 0.0f));
  const Vec3f spawnPos(0.0f, 10.0f, 0.0f);
  BodyHandle h = spawnFreeBody(sim, spawnPos, Vec3f(0.0f));

  const float fixedDt = sim.config().fixedDt;
  sim.advance(3.0f * fixedDt);
  ASSERT_EQ(sim.stepsTakenLastAdvance(), 3);

  // A buggy implementation that snapshots inside the loop on every
  // iteration would leave prevPosition equal to the state before step 3
  // (i.e. after step 2, already displaced by gravity), not the spawn state.
  Vec3f prevPos = sim.world().bodies().prevPosition(h);
  EXPECT_FLOAT_EQ(prevPos.x, spawnPos.x);
  EXPECT_FLOAT_EQ(prevPos.y, spawnPos.y);
  EXPECT_FLOAT_EQ(prevPos.z, spawnPos.z);

  EXPECT_NE(sim.world().bodies().position(h).y, spawnPos.y);
}
