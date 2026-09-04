#include <gtest/gtest.h>

#include <collision/Gjk.hpp>
#include <math/Transform.hpp>

namespace {
Transform at(float x, float y, float z = 0.0f) {
  Transform t;
  t.position = Vec3f(x, y, z);
  return t;
}
}  // namespace

TEST(GjkTest, DetectsOverlappingSpheres) {
  ShapeVariant a{SphereShape{1.0f}};
  ShapeVariant b{SphereShape{1.0f}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.5f, 0.0f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_TRUE(result.overlapping);
}

TEST(GjkTest, DetectsNonOverlappingSpheres) {
  ShapeVariant a{SphereShape{1.0f}};
  ShapeVariant b{SphereShape{1.0f}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(5.0f, 0.0f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_FALSE(result.overlapping);
}

TEST(GjkTest, DetectsOverlappingBoxes2D) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.5f, 0.0f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_TRUE(result.overlapping);
  EXPECT_EQ(result.simplexCount, 3);
}

TEST(GjkTest, DetectsSeparatedBoxes2D) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(5.0f, 0.0f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_FALSE(result.overlapping);
}

TEST(GjkTest, DetectsOverlappingHull2D) {
  std::vector<Vec3f> verts{Vec3f(-1.0f, -1.0f, 0.0f), Vec3f(1.0f, -1.0f, 0.0f),
                           Vec3f(0.0f, 1.0f, 0.0f)};
  ShapeVariant a{ConvexHullShape(verts)};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 0.0f)}};
  Transform ta = at(0.0f, 0.0f);
  Transform tb = at(1.0f, 0.0f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_TRUE(result.overlapping);
}

TEST(GjkTest, DetectsOverlappingBoxes3D) {
  ShapeVariant a{BoxShape{Vec3f(1.0f, 1.0f, 1.0f)}};
  ShapeVariant b{BoxShape{Vec3f(1.0f, 1.0f, 1.0f)}};
  Transform ta = at(0.0f, 0.0f, 0.0f);
  Transform tb = at(0.5f, 0.5f, 0.5f);

  GjkResult result = gjkOverlap(a, ta, b, tb);
  EXPECT_TRUE(result.overlapping);
  EXPECT_EQ(result.simplexCount, 4);
}
