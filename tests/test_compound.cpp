#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <collision/ManifoldCache.hpp>
#include <collision/MassProperties.hpp>
#include <collision/Shapes.hpp>
#include <core/World.hpp>

namespace {

void expectMatNear(const Mat3f& a, const Mat3f& b, float tol) {
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      EXPECT_NEAR(a[c][r], b[c][r], tol) << "col " << c << " row " << r;
    }
  }
}

Transform at(Vec3f p, Quatf q = Quatf(1.0f, 0.0f, 0.0f, 0.0f)) {
  return Transform{p, q};
}

CompoundShape dumbbell() {
  return CompoundShape({
      CompoundChild{SphereShape{0.5f}, at(Vec3f(-2.0f, 0.0f, 0.0f))},
      CompoundChild{SphereShape{0.5f}, at(Vec3f(2.0f, 0.0f, 0.0f))},
  });
}

}  // namespace

TEST(CompoundShapeTest, ChildDefaults) {
  CompoundChild child{BoxShape{Vec3f(1.0f)}};
  EXPECT_FLOAT_EQ(child.weight, 1.0f);
  EXPECT_EQ(child.localTransform.position, Vec3f(0.0f));
  EXPECT_FLOAT_EQ(child.localTransform.orientation.w, 1.0f);
  CompoundChild fromVariant{ShapeVariant{SphereShape{1.0f}}};
  EXPECT_TRUE(std::holds_alternative<SphereShape>(fromVariant.shape));
}

TEST(CompoundShapeTest, RejectsInvalidChildren) {
  EXPECT_THROW(CompoundShape(std::vector<CompoundChild>{}),
               std::invalid_argument);
  EXPECT_THROW(CompoundShape({CompoundChild{SphereShape{1.0f}, {}, -1.0f}}),
               std::invalid_argument);
  EXPECT_THROW(CompoundShape({CompoundChild{SphereShape{1.0f}, {}, 0.0f}}),
               std::invalid_argument);
  EXPECT_THROW(
      CompoundShape({CompoundChild{SphereShape{1.0f}, {}, std::nanf("")}}),
      std::invalid_argument);
  EXPECT_THROW(CompoundShape({CompoundChild{SphereShape{1.0f}, {},
                                            INFINITY}}),
               std::invalid_argument);
  ShapeVariant nested{dumbbell()};
  EXPECT_THROW(CompoundShape({CompoundChild{nested}}), std::invalid_argument);
}

TEST(CompoundShapeTest, SupportBoundsAndAABB) {
  CompoundShape c = dumbbell();
  Vec3f s = c.support(Vec3f(1.0f, 0.0f, 0.0f));
  EXPECT_NEAR(s.x, 2.5f, 1e-5f);
  s = c.support(Vec3f(-1.0f, 0.0f, 0.0f));
  EXPECT_NEAR(s.x, -2.5f, 1e-5f);
  EXPECT_NEAR(c.boundingRadius(), 2.5f, 1e-5f);
  AABB box = c.localAABB();
  EXPECT_NEAR(box.min.x, -2.5f, 1e-5f);
  EXPECT_NEAR(box.max.y, 0.5f, 1e-5f);

  Transform t = at(Vec3f(10.0f, 0.0f, 0.0f),
                   glm::angleAxis(glm::radians(90.0f), Vec3f(0, 0, 1)));
  ShapeVariant v{c};
  AABB w = worldAABB(v, t);
  EXPECT_NEAR(w.min.x, 9.5f, 1e-4f);
  EXPECT_NEAR(w.max.x, 10.5f, 1e-4f);
  EXPECT_NEAR(w.min.y, -2.5f, 1e-4f);
  EXPECT_NEAR(w.max.y, 2.5f, 1e-4f);
  Vec3f ws = worldSupport(v, t, Vec3f(0.0f, 1.0f, 0.0f));
  EXPECT_NEAR(ws.y, 2.5f, 1e-4f);
  EXPECT_NEAR(ws.x, 10.0f, 1e-4f);
}

TEST(CompoundShapeTest, InertiaParallelAxisAndWeights) {
  // spheres of mass 1 and 3 at x=-2 and x=+2
  CompoundShape c({
      CompoundChild{SphereShape{0.5f}, at(Vec3f(-2.0f, 0.0f, 0.0f)), 1.0f},
      CompoundChild{SphereShape{0.5f}, at(Vec3f(2.0f, 0.0f, 0.0f)), 3.0f},
  });
  Mat3f inertia = c.localInertiaTensor(4.0f);
  float iSphere1 = 0.4f * 1.0f * 0.25f, iSphere3 = 0.4f * 3.0f * 0.25f;
  EXPECT_NEAR(inertia[0][0], iSphere1 + iSphere3, 1e-5f);
  EXPECT_NEAR(inertia[1][1], iSphere1 + iSphere3 + 4.0f * 4.0f, 1e-5f);
  EXPECT_NEAR(inertia[2][2], iSphere1 + iSphere3 + 4.0f * 4.0f, 1e-5f);

  MassProperties mp = computeMassProperties(ShapeVariant{c}, 4.0f);
  expectMatNear(mp.localInertiaTensor * mp.invLocalInertiaTensor, Mat3f(1.0f),
                1e-4f);
}

TEST(CompoundShapeTest, OffDiagonalInertiaAndFullInverse) {
  // single point-ish sphere offset along (1,1,0): products of inertia appear
  CompoundShape c({CompoundChild{BoxShape{Vec3f(0.5f, 0.2f, 0.3f)},
                                 at(Vec3f(1.0f, 1.0f, 0.5f),
                                    glm::angleAxis(0.7f, glm::normalize(
                                                             Vec3f(1, 2, 3))))},
                   CompoundChild{CapsuleShape{0.2f, 0.4f},
                                 at(Vec3f(-0.5f, 0.2f, -0.3f)), 2.0f}});
  MassProperties mp = computeMassProperties(ShapeVariant{c}, 3.0f);
  const Mat3f& I = mp.localInertiaTensor;
  // compare against manual reference
  BoxShape b{Vec3f(0.5f, 0.2f, 0.3f)};
  Mat3f r = glm::mat3_cast(glm::angleAxis(0.7f, glm::normalize(Vec3f(1, 2, 3))));
  Vec3f d1(1.0f, 1.0f, 0.5f), d2(-0.5f, 0.2f, -0.3f);
  Mat3f ref = r * b.localInertiaTensor(1.0f) * glm::transpose(r) +
              1.0f * (Mat3f(glm::dot(d1, d1)) - glm::outerProduct(d1, d1)) +
              CapsuleShape{0.2f, 0.4f}.localInertiaTensor(2.0f) +
              2.0f * (Mat3f(glm::dot(d2, d2)) - glm::outerProduct(d2, d2));
  expectMatNear(I, ref, 1e-4f);
  EXPECT_GT(std::abs(I[0][1]), 1e-3f);
  expectMatNear(I * mp.invLocalInertiaTensor, Mat3f(1.0f), 1e-4f);
  expectMatNear(mp.invLocalInertiaTensor * I, Mat3f(1.0f), 1e-4f);

  MassProperties stat = computeMassProperties(ShapeVariant{c}, 0.0f);
  expectMatNear(stat.invLocalInertiaTensor, Mat3f(0.0f), 0.0f);
}

TEST(CompoundShapeTest, SingleCenteredChildMatchesLeaf) {
  CompoundShape c({CompoundChild{BoxShape{Vec3f(1.0f, 2.0f, 3.0f)}}});
  expectMatNear(c.localInertiaTensor(5.0f),
                BoxShape{Vec3f(1.0f, 2.0f, 3.0f)}.localInertiaTensor(5.0f),
                1e-5f);
}

TEST(CompoundShapeTest, CenterOfMassAndRecentering) {
  // sphere of weight 1 at x=-2 and sphere of weight 3 at x=+2 -> CoM at x=+1
  CompoundShape raw({
      CompoundChild{SphereShape{0.5f}, at(Vec3f(-2.0f, 0.0f, 0.0f)), 1.0f},
      CompoundChild{SphereShape{0.5f}, at(Vec3f(2.0f, 0.0f, 0.0f)), 3.0f},
  });
  Vec3f com = raw.centerOfMass();
  EXPECT_NEAR(com.x, 1.0f, 1e-5f);
  EXPECT_NEAR(com.y, 0.0f, 1e-5f);
  EXPECT_NEAR(com.z, 0.0f, 1e-5f);

  CompoundShape centered = raw.recentered();
  EXPECT_NEAR(glm::length(centered.centerOfMass()), 0.0f, 1e-6f);
  EXPECT_NEAR(centered.children[0].localTransform.position.x, -3.0f, 1e-5f);
  EXPECT_NEAR(centered.children[1].localTransform.position.x, 1.0f, 1e-5f);

  // parallel-axis theorem: I_raw = I_com + M * (|com|^2 E - com * com^T)
  const float mass = 4.0f;
  Mat3f shift =
      mass * (Mat3f(glm::dot(com, com)) - glm::outerProduct(com, com));
  expectMatNear(centered.localInertiaTensor(mass) + shift,
                raw.localInertiaTensor(mass), 1e-4f);
}

TEST(CompoundWorldTest, CompoundRestsOnGroundWithBothEnds) {
  World world;
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 20.0f)}},
                         at(Vec3f(0.0f, -0.5f, 0.0f)), 0.5f, 0.0f);
  BodyHandle h = world.createDynamicBody(ShapeVariant{dumbbell()},
                                         at(Vec3f(0.0f, 2.0f, 0.0f)), 2.0f,
                                         0.5f, 0.0f);
  for (int i = 0; i < 300; ++i) { world.step(1.0f / 60.0f); }
  const BodyStore& b = world.bodies();
  // both ends support the body: it stays level at sphere-radius height
  EXPECT_NEAR(b.position(h).y, 0.5f, 0.05f);
  EXPECT_LT(std::abs(b.linearVelocity(h).y), 0.05f);
  Vec3f axis = b.orientation(h) * Vec3f(1.0f, 0.0f, 0.0f);
  EXPECT_LT(std::abs(axis.y), 0.02f);
}

TEST(CompoundWorldTest, BoxEndedBarComesToRest) {
  World world;
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 20.0f)}},
                         at(Vec3f(0.0f, -0.5f, 0.0f)), 0.5f, 0.0f);
  CompoundShape bar({
      CompoundChild{BoxShape{Vec3f(0.5f)}, at(Vec3f(-2.0f, 0.0f, 0.0f)), 2.0f},
      CompoundChild{BoxShape{Vec3f(1.5f, 0.1f, 0.1f)}, at(Vec3f(0.0f)), 0.5f},
      CompoundChild{BoxShape{Vec3f(0.5f)}, at(Vec3f(2.0f, 0.0f, 0.0f)), 1.0f},
  });
  BodyHandle h = world.createDynamicBody(
      ShapeVariant{bar},
      at(Vec3f(0.0f, 2.0f, 0.0f),
         glm::angleAxis(0.2f, glm::normalize(Vec3f(0.3f, 0.0f, 1.0f)))),
      3.0f, 0.6f, 0.0f);
  for (int i = 0; i < 400; ++i) { world.step(1.0f / 60.0f); }
  const BodyStore& b = world.bodies();
  EXPECT_NEAR(b.position(h).y, 0.5f, 0.03f);
  EXPECT_LT(glm::length(b.linearVelocity(h)), 0.02f);
  EXPECT_LT(glm::length(b.angularVelocity(h)), 0.02f);
  Vec3f up = b.orientation(h) * Vec3f(0.0f, 1.0f, 0.0f);
  EXPECT_GT(up.y, 0.999f);
}

TEST(CompoundWorldTest, SphereFallsThroughGapOfNonConvexCompound) {
  // two pillars with a wide gap: as a convex hull it would block the sphere,
  // as a compound the sphere falls between the pillars
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));
  CompoundShape pillars({
      CompoundChild{BoxShape{Vec3f(0.5f, 0.5f, 3.0f)}, at(Vec3f(-3.0f, 0, 0))},
      CompoundChild{BoxShape{Vec3f(0.5f, 0.5f, 3.0f)}, at(Vec3f(3.0f, 0, 0))},
  });
  world.createStaticBody(ShapeVariant{pillars}, at(Vec3f(0.0f)), 0.5f, 0.0f);
  BodyHandle s = world.createDynamicBody(ShapeVariant{SphereShape{0.5f}},
                                         at(Vec3f(0.0f, 3.0f, 0.0f)), 1.0f,
                                         0.5f, 0.0f);
  for (int i = 0; i < 120; ++i) { world.step(1.0f / 60.0f); }
  EXPECT_LT(world.bodies().position(s).y, -2.0f);
}

TEST(CompoundWorldTest, NarrowphaseOnlyRunsOnOverlappingChildPairs) {
  World world;
  world.setGravity(Vec3f(0.0f));
  int gjkCalls = 0;
  world.setNarrowphaseFns(
      [&](const ShapeVariant& a, const Transform& ta, const ShapeVariant& b,
          const Transform& tb) {
        ++gjkCalls;
        EXPECT_FALSE(isCompound(a));
        EXPECT_FALSE(isCompound(b));
        return gjkOverlap(a, ta, b, tb);
      },
      nullptr);
  world.createStaticBody(ShapeVariant{dumbbell()}, at(Vec3f(0.0f)), 0.5f,
                         0.0f);
  // sphere touching only the right end
  world.createDynamicBody(ShapeVariant{SphereShape{0.5f}},
                          at(Vec3f(2.0f, 0.9f, 0.0f)), 1.0f, 0.5f, 0.0f);
  world.step(1.0f / 60.0f);
  EXPECT_EQ(gjkCalls, 1);
}

TEST(CompoundWorldTest, CompoundVsCompoundStacksStably) {
  World world;
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 20.0f)}},
                         at(Vec3f(0.0f, -0.5f, 0.0f)), 0.5f, 0.0f);
  CompoundShape table({
      CompoundChild{BoxShape{Vec3f(1.0f, 0.1f, 1.0f)}, at(Vec3f(0, 0.5f, 0))},
      CompoundChild{BoxShape{Vec3f(0.1f, 0.4f, 0.1f)},
                    at(Vec3f(-0.8f, 0.0f, -0.8f))},
      CompoundChild{BoxShape{Vec3f(0.1f, 0.4f, 0.1f)},
                    at(Vec3f(0.8f, 0.0f, -0.8f))},
      CompoundChild{BoxShape{Vec3f(0.1f, 0.4f, 0.1f)},
                    at(Vec3f(-0.8f, 0.0f, 0.8f))},
      CompoundChild{BoxShape{Vec3f(0.1f, 0.4f, 0.1f)},
                    at(Vec3f(0.8f, 0.0f, 0.8f))},
  });
  BodyHandle t1 = world.createDynamicBody(ShapeVariant{table},
                                          at(Vec3f(0.0f, 0.45f, 0.0f)), 4.0f,
                                          0.6f, 0.0f);
  BodyHandle t2 = world.createDynamicBody(ShapeVariant{table},
                                          at(Vec3f(0.0f, 1.5f, 0.0f)), 4.0f,
                                          0.6f, 0.0f);
  for (int i = 0; i < 400; ++i) { world.step(1.0f / 60.0f); }
  const BodyStore& b = world.bodies();
  EXPECT_NEAR(b.position(t1).y, 0.4f, 0.05f);
  EXPECT_NEAR(b.position(t2).y, 1.4f, 0.08f);
  EXPECT_NEAR(b.position(t2).x, 0.0f, 0.05f);
  EXPECT_LT(glm::length(b.linearVelocity(t2)), 0.05f);
}

TEST(CompoundManifoldCacheTest, ChildPairsGetIndependentWarmStartedManifolds) {
  ManifoldCache cache;
  BodyHandle a{0, 0}, b{1, 0};
  auto make = [&](uint32_t ca, uint32_t cb, Vec3f anchor, float impulse) {
    Manifold m{};
    m.bodyA = a;
    m.bodyB = b;
    m.childIndexA = ca;
    m.childIndexB = cb;
    m.normal = Vec3f(0.0f, 1.0f, 0.0f);
    m.points[0].localAnchorA = anchor;
    m.points[0].localAnchorB = anchor;
    m.points[0].normalImpulse = impulse;
    m.pointCount = 1;
    return m;
  };

  cache.beginFrame();
  cache.updateManifold(make(0, 0, Vec3f(-2.0f, 0.0f, 0.0f), 3.0f), 0.1f);
  cache.updateManifold(make(1, 0, Vec3f(2.0f, 0.0f, 0.0f), 7.0f), 0.1f);
  cache.endFrame();
  ASSERT_EQ(cache.activeManifolds().size(), 2u);

  // next frame: fresh manifolds carry no impulse; cache restores per child
  cache.beginFrame();
  cache.updateManifold(make(0, 0, Vec3f(-2.0f, 0.0f, 0.01f), 0.0f), 0.1f);
  cache.updateManifold(make(1, 0, Vec3f(2.0f, 0.0f, 0.01f), 0.0f), 0.1f);
  cache.endFrame();
  ASSERT_EQ(cache.activeManifolds().size(), 2u);
  EXPECT_FLOAT_EQ(cache.getOrCreate(a, b, 0, 0).points[0].normalImpulse, 3.0f);
  EXPECT_FLOAT_EQ(cache.getOrCreate(b, a, 0, 1).points[0].normalImpulse, 7.0f);

  // a child pair that stops touching is evicted independently
  cache.beginFrame();
  cache.updateManifold(make(1, 0, Vec3f(2.0f, 0.0f, 0.0f), 0.0f), 0.1f);
  cache.endFrame();
  ASSERT_EQ(cache.activeManifolds().size(), 1u);
  EXPECT_EQ(cache.activeManifolds()[0].childIndexA, 1u);

  // reversed body order (b, a) with swapped child indices (0, 1) matches
  // newAnchor (in b's frame) against oldSnapshot.localAnchorB and carries over
  // normalImpulse via the swapped path
  Manifold& cachedEntry = cache.getOrCreate(a, b, 1, 0);
  cachedEntry.points[0].localAnchorA = Vec3f(2.0f, 0.0f, 0.0f);
  cachedEntry.points[0].localAnchorB = Vec3f(-5.0f, 1.0f, 0.0f);
  cachedEntry.points[0].normalImpulse = 9.5f;

  cache.beginFrame();
  Manifold reversed{};
  reversed.bodyA = b;
  reversed.bodyB = a;
  reversed.childIndexA = 0;
  reversed.childIndexB = 1;
  reversed.normal = Vec3f(0.0f, -1.0f, 0.0f);
  reversed.points[0].localAnchorA = Vec3f(-5.0f, 1.0f, 0.01f);
  reversed.points[0].localAnchorB = Vec3f(2.0f, 0.0f, 0.01f);
  reversed.points[0].normalImpulse = 0.0f;
  reversed.pointCount = 1;
  cache.updateManifold(reversed, 0.1f);
  cache.endFrame();
  ASSERT_EQ(cache.activeManifolds().size(), 1u);
  EXPECT_FLOAT_EQ(cache.getOrCreate(a, b, 1, 0).points[0].normalImpulse, 9.5f);
}

TEST(CompoundWorldTest, RecenteredAsymmetricBarTipsTowardHeavyEndOnFulcrum) {
  World world;
  // narrow fulcrum centered at x = 0, top surface at y = 0
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(0.15f, 0.5f, 1.0f)}},
                         at(Vec3f(0.0f, -0.5f, 0.0f)), 0.8f, 0.0f);

  // asymmetric bar: light left end (w=1 at x=-1.5), heavy right end (w=5 at
  // x=+1.5). Recentering shifts local (0,0,0) to x = +1.0 (the true CoM).
  CompoundShape rawBar({
      CompoundChild{BoxShape{Vec3f(2.0f, 0.1f, 0.2f)}, at(Vec3f(0.0f)), 1.0f},
      CompoundChild{SphereShape{0.2f}, at(Vec3f(-1.5f, 0.0f, 0.0f)), 1.0f},
      CompoundChild{SphereShape{0.2f}, at(Vec3f(1.5f, 0.0f, 0.0f)), 5.0f},
  });
  Vec3f comOffset = rawBar.centerOfMass();
  ASSERT_GT(comOffset.x, 0.5f);

  CompoundShape centeredBar = rawBar.recentered();
  // place the bar so its geometric center is at x = 0 (directly over the
  // fulcrum), which means its body position (CoM) starts at +comOffset
  BodyHandle h = world.createDynamicBody(
      ShapeVariant{centeredBar},
      at(Vec3f(comOffset.x, 0.12f, 0.0f)), 3.5f, 0.8f, 0.0f);

  for (int i = 0; i < 60; ++i) { world.step(1.0f / 60.0f); }
  const BodyStore& b = world.bodies();
  // right (+X) end is heavier, so the bar rotates clockwise (negative Z spin,
  // +X axis tips downward in Y)
  Vec3f rightAxis = b.orientation(h) * Vec3f(1.0f, 0.0f, 0.0f);
  EXPECT_LT(rightAxis.y, -0.2f);
}
