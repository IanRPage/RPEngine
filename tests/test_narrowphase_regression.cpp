#include <gtest/gtest.h>

#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <math/Transform.hpp>
#include <vector>

namespace {

constexpr float kTol = 1e-3f;

struct NarrowphaseResult {
  bool colliding;
  float penetration;
  Vec3f normal;
};

NarrowphaseResult narrowphase(const ShapeVariant& a, const Transform& ta,
                              const ShapeVariant& b, const Transform& tb) {
  GjkResult gjk = gjkOverlap(a, ta, b, tb);
  if (!gjk.overlapping) return {false, 0.0f, Vec3f(0.0f)};
  EpaResult epa = epaPenetration(a, ta, b, tb, gjk);
  return {true, epa.penetrationDepth, epa.normal};
}

Transform at(float x, float y) {
  Transform t;
  t.position = Vec3f(x, y, 0.0f);
  return t;
}

ShapeVariant sphere(float radius) { return ShapeVariant{SphereShape{radius}}; }

ShapeVariant box(float width, float height) {
  return ShapeVariant{BoxShape{Vec3f(width * 0.5f, height * 0.5f, 20.0f)}};
}

ShapeVariant triangle(Vec2f v0, Vec2f v1, Vec2f v2, Transform& outTransform) {
  Vec2f c = (v0 + v1 + v2) / 3.0f;
  outTransform.position = Vec3f(c.x, c.y, 0.0f);
  std::vector<Vec3f> verts{Vec3f(v0.x - c.x, v0.y - c.y, 0.0f),
                           Vec3f(v1.x - c.x, v1.y - c.y, 0.0f),
                           Vec3f(v2.x - c.x, v2.y - c.y, 0.0f)};
  return ShapeVariant{ConvexHullShape(verts)};
}

}  // namespace

// --- Circle vs Rectangle ---

TEST(NarrowphaseRegressionTest, CircleVsRectangle_DeepIntersection) {
  ShapeVariant circle = sphere(10.0f);
  Transform tc = at(0.0f, 0.0f);
  ShapeVariant rect = box(10.0f, 10.0f);
  Transform tr = at(5.0f, 0.0f);  // x in [0, 10], y in [-5, 5]

  auto result = narrowphase(circle, tc, rect, tr);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 10.0f, kTol);
  EXPECT_NEAR(result.normal.x, 1.0f, kTol);
  EXPECT_NEAR(result.normal.y, 0.0f, kTol);

  auto flipped = narrowphase(rect, tr, circle, tc);
  EXPECT_TRUE(flipped.colliding);
  EXPECT_NEAR(flipped.penetration, 10.0f, kTol);
  EXPECT_NEAR(flipped.normal.x, -1.0f, kTol);
  EXPECT_NEAR(flipped.normal.y, 0.0f, kTol);
}

TEST(NarrowphaseRegressionTest, CircleVsRectangle_EdgeIntersection) {
  ShapeVariant circle = sphere(5.0f);
  Transform tc = at(0.0f, 0.0f);
  ShapeVariant rect = box(6.0f, 6.0f);
  Transform tr = at(6.0f, 0.0f);  // x in [3, 9], y in [-3, 3]

  auto result = narrowphase(circle, tc, rect, tr);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 2.0f, kTol);
}

TEST(NarrowphaseRegressionTest, CircleVsRectangle_CornerIntersection) {
  ShapeVariant circle = sphere(6.0f);
  Transform tc = at(0.0f, 0.0f);
  ShapeVariant rect = box(4.0f, 4.0f);
  Transform tr = at(5.0f, 6.0f);  // min corner at (3, 4)

  auto result = narrowphase(circle, tc, rect, tr);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, CircleVsRectangle_VertexToVertex) {
  ShapeVariant circle = sphere(2.0f);
  Transform tc = at(0.0f, 0.0f);
  ShapeVariant rect = box(2.0f, 2.0f);
  Transform tr = at(2.0f, 2.0f);

  auto result = narrowphase(circle, tc, rect, tr);
  EXPECT_TRUE(result.colliding);
  EXPECT_GT(result.penetration, 0.0f);
}

TEST(NarrowphaseRegressionTest, CircleVsRectangle_FalsePositive) {
  ShapeVariant circle = sphere(5.0f);
  Transform tc = at(0.0f, 0.0f);
  ShapeVariant rect = box(10.0f, 10.0f);
  Transform tr = at(11.0f, 0.0f);  // min x at 6, gap of 1

  auto result = narrowphase(circle, tc, rect, tr);
  EXPECT_FALSE(result.colliding);
}

// --- Circle vs Triangle ---

TEST(NarrowphaseRegressionTest, CircleVsTriangle_DeepIntersection) {
  ShapeVariant circle = sphere(5.0f);
  Transform tc = at(0.0f, 0.0f);
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{-6.0f, -6.0f}, Vec2f{6.0f, -6.0f},
                              Vec2f{0.0f, 6.0f}, tt);

  auto result = narrowphase(circle, tc, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_GT(result.penetration, 0.0f);
}

TEST(NarrowphaseRegressionTest, CircleVsTriangle_EdgeIntersection) {
  ShapeVariant circle = sphere(3.0f);
  Transform tc = at(0.0f, -2.0f);
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{-5.0f, 0.0f}, Vec2f{5.0f, 0.0f},
                              Vec2f{0.0f, 10.0f}, tt);

  auto result = narrowphase(circle, tc, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, CircleVsTriangle_VertexIntersection) {
  ShapeVariant circle = sphere(2.0f);
  Transform tc = at(0.0f, -1.0f);
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{0.0f, 0.0f}, Vec2f{5.0f, 5.0f},
                              Vec2f{-5.0f, 5.0f}, tt);

  auto result = narrowphase(circle, tc, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, 1e-2f);
}

TEST(NarrowphaseRegressionTest, CircleVsTriangle_FalsePositive) {
  ShapeVariant circle = sphere(2.0f);
  Transform tc = at(0.0f, -3.0f);
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{0.0f, 0.0f}, Vec2f{5.0f, 5.0f},
                              Vec2f{-5.0f, 5.0f}, tt);

  auto result = narrowphase(circle, tc, tri, tt);
  EXPECT_FALSE(result.colliding);
}

// --- Rectangle vs Rectangle ---

TEST(NarrowphaseRegressionTest, RectangleVsRectangle_DeepIntersection) {
  ShapeVariant r1 = box(10.0f, 10.0f);
  Transform t1 = at(0.0f, 0.0f);
  ShapeVariant r2 = box(10.0f, 10.0f);
  Transform t2 = at(2.0f, 0.0f);

  auto result = narrowphase(r1, t1, r2, t2);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 8.0f, kTol);
}

TEST(NarrowphaseRegressionTest, RectangleVsRectangle_EdgeIntersection) {
  ShapeVariant r1 = box(10.0f, 10.0f);
  Transform t1 = at(0.0f, 0.0f);
  ShapeVariant r2 = box(10.0f, 10.0f);
  Transform t2 = at(9.0f, 0.0f);

  auto result = narrowphase(r1, t1, r2, t2);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, RectangleVsRectangle_CornerIntersection) {
  ShapeVariant r1 = box(10.0f, 10.0f);
  Transform t1 = at(0.0f, 0.0f);
  ShapeVariant r2 = box(10.0f, 10.0f);
  Transform t2 = at(8.0f, 8.0f);

  auto result = narrowphase(r1, t1, r2, t2);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 2.0f, kTol);
}

TEST(NarrowphaseRegressionTest, RectangleVsRectangle_VertexToVertex) {
  ShapeVariant r1 = box(10.0f, 10.0f);
  Transform t1 = at(0.0f, 0.0f);
  ShapeVariant r2 = box(10.0f, 10.0f);
  Transform t2 = at(9.0f, 9.0f);

  auto result = narrowphase(r1, t1, r2, t2);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, RectangleVsRectangle_FalsePositive) {
  ShapeVariant r1 = box(10.0f, 10.0f);
  Transform t1 = at(0.0f, 0.0f);
  ShapeVariant r2 = box(10.0f, 10.0f);
  Transform t2 = at(15.0f, 0.0f);

  auto result = narrowphase(r1, t1, r2, t2);
  EXPECT_FALSE(result.colliding);
}

// --- Rectangle vs Triangle ---

TEST(NarrowphaseRegressionTest, RectangleVsTriangle_DeepIntersection) {
  ShapeVariant rect = box(10.0f, 10.0f);
  Transform tr = at(0.0f, 0.0f);
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{-6.0f, -6.0f}, Vec2f{6.0f, -6.0f},
                              Vec2f{0.0f, 6.0f}, tt);

  auto result = narrowphase(rect, tr, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_GT(result.penetration, 0.0f);
}

TEST(NarrowphaseRegressionTest, RectangleVsTriangle_EdgeIntersection) {
  ShapeVariant rect = box(6.0f, 6.0f);
  Transform tr = at(0.0f, -3.0f);  // top edge at y=0
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{-5.0f, -1.0f}, Vec2f{5.0f, -1.0f},
                              Vec2f{0.0f, 10.0f}, tt);  // bottom edge y=-1

  auto result = narrowphase(rect, tr, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, RectangleVsTriangle_VertexToVertex) {
  ShapeVariant rect = box(6.0f, 6.0f);
  Transform tr = at(0.0f, -3.0f);  // top-right corner at (3, 0)
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{2.5f, -0.5f}, Vec2f{8.0f, -1.0f},
                              Vec2f{5.0f, 5.0f}, tt);

  auto result = narrowphase(rect, tr, tri, tt);
  EXPECT_TRUE(result.colliding);
  EXPECT_GT(result.penetration, 0.0f);
}

TEST(NarrowphaseRegressionTest, RectangleVsTriangle_FalsePositive) {
  ShapeVariant rect = box(6.0f, 6.0f);
  Transform tr = at(0.0f, -5.0f);  // top edge at y=-2
  Transform tt;
  ShapeVariant tri = triangle(Vec2f{-5.0f, 0.0f}, Vec2f{5.0f, 0.0f},
                              Vec2f{0.0f, 10.0f}, tt);  // bottom edge y=0

  auto result = narrowphase(rect, tr, tri, tt);
  EXPECT_FALSE(result.colliding);
}

// --- Triangle vs Triangle ---

TEST(NarrowphaseRegressionTest, TriangleVsTriangle_DeepIntersection) {
  Transform t1t, t2t;
  ShapeVariant t1 = triangle(Vec2f{-5.0f, -5.0f}, Vec2f{5.0f, -5.0f},
                             Vec2f{0.0f, 5.0f}, t1t);
  ShapeVariant t2 = triangle(Vec2f{-5.0f, 5.0f}, Vec2f{5.0f, 5.0f},
                             Vec2f{0.0f, -5.0f}, t2t);

  auto result = narrowphase(t1, t1t, t2, t2t);
  EXPECT_TRUE(result.colliding);
  EXPECT_GT(result.penetration, 0.0f);
}

TEST(NarrowphaseRegressionTest, TriangleVsTriangle_EdgeIntersection) {
  Transform t1t, t2t;
  ShapeVariant t1 = triangle(Vec2f{-5.0f, 0.0f}, Vec2f{5.0f, 0.0f},
                             Vec2f{0.0f, 10.0f}, t1t);
  ShapeVariant t2 = triangle(Vec2f{-5.0f, -5.0f}, Vec2f{5.0f, -5.0f},
                             Vec2f{0.0f, 1.0f}, t2t);

  auto result = narrowphase(t1, t1t, t2, t2t);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 1.0f, kTol);
}

TEST(NarrowphaseRegressionTest, TriangleVsTriangle_VertexToVertex) {
  Transform t1t, t2t;
  ShapeVariant t1 = triangle(Vec2f{-5.0f, -5.0f}, Vec2f{5.0f, -5.0f},
                             Vec2f{0.0f, 1.0f}, t1t);
  ShapeVariant t2 = triangle(Vec2f{-5.0f, 5.0f}, Vec2f{5.0f, 5.0f},
                             Vec2f{0.0f, 0.0f}, t2t);

  auto result = narrowphase(t1, t1t, t2, t2t);
  EXPECT_TRUE(result.colliding);
  EXPECT_NEAR(result.penetration, 0.70710678f, kTol);
}

TEST(NarrowphaseRegressionTest, TriangleVsTriangle_FalsePositive) {
  Transform t1t, t2t;
  ShapeVariant t1 = triangle(Vec2f{-5.0f, -5.0f}, Vec2f{5.0f, -5.0f},
                             Vec2f{0.0f, 0.0f}, t1t);
  ShapeVariant t2 = triangle(Vec2f{-5.0f, 5.0f}, Vec2f{5.0f, 5.0f},
                             Vec2f{0.0f, 1.0f}, t2t);

  auto result = narrowphase(t1, t1t, t2, t2t);
  EXPECT_FALSE(result.colliding);
}
