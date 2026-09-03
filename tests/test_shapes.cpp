#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <collision/Shapes.hpp>

TEST(BoxShapeTest, SupportReturnsCorrectCorner) {
  BoxShape box{Vec3f(2.0f, 3.0f, 4.0f)};

  Vec3f a = box.support(Vec3f(1.0f, 1.0f, 1.0f));
  EXPECT_FLOAT_EQ(a.x, 2.0f);
  EXPECT_FLOAT_EQ(a.y, 3.0f);
  EXPECT_FLOAT_EQ(a.z, 4.0f);

  Vec3f b = box.support(Vec3f(-1.0f, 1.0f, -1.0f));
  EXPECT_FLOAT_EQ(b.x, -2.0f);
  EXPECT_FLOAT_EQ(b.y, 3.0f);
  EXPECT_FLOAT_EQ(b.z, -4.0f);
}

TEST(BoxShapeTest, InertiaMatchesTextbookFormula) {
  // 2x2x2 cube (half-extents 1,1,1), mass 6
  // Ixx = m/3 * (hy^2+hz^2) = 6/3 * (1+1) = 4, Symmetric for Iyy & Izz
  BoxShape box{Vec3f(1.0f, 1.0f, 1.0f)};
  Mat3f i = box.localInertiaTensor(6.0f);

  EXPECT_NEAR(i[0][0], 4.0f, 1e-4f);
  EXPECT_NEAR(i[1][1], 4.0f, 1e-4f);
  EXPECT_NEAR(i[2][2], 4.0f, 1e-4f);
}

TEST(SphereShapeTest, InertiaMatchesTextbookFormula) {
  // I = (2/5) * m * r^2 = (2/5) * 5 * 4 = 8
  SphereShape sphere{2.0f};
  Mat3f i = sphere.localInertiaTensor(5.0f);

  EXPECT_NEAR(i[0][0], 8.0f, 1e-4f);
  EXPECT_NEAR(i[1][1], 8.0f, 1e-4f);
  EXPECT_NEAR(i[2][2], 8.0f, 1e-4f);
}

TEST(CapsuleShapeTest, InertiaMatchesReferenceValue) {
  // mass=10, radius=1, halfHeight=2:
  //   Vcyl = pi*r^2*2h = 4pi, Vcaps = (4/3)*pi*r^3 = (4/3)pi
  //   mc = mass * Vcyl/(Vcyl+Vcaps) = 10 * 0.75 = 7.5, ms = 2.5
  //   Iyy = mc*r^2/2 + ms*(2/5)r^2 = 3.75 + 1.0 = 4.75
  //   Ixx = mc*(h^2/3 + r^2/4) + ms*((2/5)r^2 + h^2 + (3/4)*h*r)
  //       = 7.5*(19/12) + 2.5*5.9 = 11.875 + 14.75 = 26.625
  CapsuleShape capsule{1.0f, 2.0f};
  Mat3f i = capsule.localInertiaTensor(10.0f);

  EXPECT_NEAR(i[0][0], 26.625f, 1e-3f);
  EXPECT_NEAR(i[1][1], 4.75f, 1e-3f);
  EXPECT_NEAR(i[2][2], 26.625f, 1e-3f);
}

TEST(CapsuleShapeTest, SupportNormalizesFullDirectionNotJustRadialProjection) {
  CapsuleShape capsule{2.0f, 3.0f};
  Vec3f direction(1.0f, 1.0f, 0.0f);
  Vec3f point = capsule.support(direction);

  float invSqrt2 = 1.0f / std::sqrt(2.0f);
  EXPECT_NEAR(point.x, 2.0f * invSqrt2, 1e-4f);
  EXPECT_NEAR(point.y, 3.0f + 2.0f * invSqrt2, 1e-4f);
  EXPECT_NEAR(point.z, 0.0f, 1e-4f);

  Vec3f buggyPoint(2.0f, 3.0f, 0.0f);
  EXPECT_GT(glm::dot(point, direction), glm::dot(buggyPoint, direction));
}

TEST(ConvexHullShapeTest, RejectsEmptyVertices) {
  EXPECT_THROW({ ConvexHullShape hull({}); }, std::invalid_argument);
}

TEST(ConvexHullShapeTest, MatchesOldTriangleBoundingRadius) {
  ConvexHullShape hull({Vec3f(-6.0f, -4.0f, 0.0f), Vec3f(6.0f, -4.0f, 0.0f),
                        Vec3f(0.0f, 8.0f, 0.0f)});

  EXPECT_NEAR(hull.boundingRadius(), 8.0f, 1e-4f);
}

TEST(ConvexHullShapeTest, InertiaMatchesHandComputedPolygonValue) {
  // same triangle from MatchesOldTriangleBoundingRadius, mass=1, area=72.
  // computed via polygon second-moment-of-area formula:
  //   Iy_area = (1/12) sum cross_i*(x_i^2 + x_i*x_{i+1} + x_{i+1}^2
  //           = 432 -> Iyy = mass/area * Iy_area = 432/72 = 6
  //   Ix_area = 576 -> Ixx = mass/area * Ix_area = 576/72 = 8
  //   Izz = Ixx + Iyy = 14 (perpendicular axis theorem)
  ConvexHullShape hull({Vec3f(-6.0f, -4.0f, 0.0f), Vec3f(6.0f, -4.0f, 0.0f),
                        Vec3f(0.0f, 8.0f, 0.0f)});
  Mat3f i = hull.localInertiaTensor(1.0f);

  EXPECT_NEAR(i[0][0], 8.0f, 1e-3f);
  EXPECT_NEAR(i[1][1], 6.0f, 1e-3f);
  EXPECT_NEAR(i[2][2], 14.0f, 1e-3f);
}
