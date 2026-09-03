#include <gtest/gtest.h>

#include <cmath>

#include <collision/MassProperties.hpp>

TEST(MassPropertiesTest, ZeroMassIsStatic) {
  SphereShape sphere{2.0f};
  MassProperties props = computeMassProperties(ShapeVariant{sphere}, 0.0f);

  EXPECT_FLOAT_EQ(props.mass, 0.0f);
  EXPECT_FLOAT_EQ(props.invMass, 0.0f);
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[0][0], 0.0f);
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[1][1], 0.0f);
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[2][2], 0.0f);
}

TEST(MassPropertiesTest, NonZeroMassInvertsDiagonal) {
  // sphere, mass=5, radius=2 -> I = (2/5)*5*4 = 8 on each diagonal entry
  SphereShape sphere{2.0f};
  MassProperties props = computeMassProperties(ShapeVariant{sphere}, 5.0f);

  EXPECT_FLOAT_EQ(props.invMass, 1.0f / 5.0f);
  EXPECT_NEAR(props.localInertiaTensor[0][0], 8.0f, 1e-4f);
  EXPECT_NEAR(props.invLocalInertiaTensor[0][0], 1.0f / 8.0f, 1e-4f);
  EXPECT_NEAR(props.invLocalInertiaTensor[1][1], 1.0f / 8.0f, 1e-4f);
  EXPECT_NEAR(props.invLocalInertiaTensor[2][2], 1.0f / 8.0f, 1e-4f);
}

TEST(MassPropertiesTest, DegenerateHullWithPositiveMassDoesNotProduceInf) {
  ConvexHullShape collinearHull(
      {Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)});
  MassProperties props = computeMassProperties(ShapeVariant{collinearHull}, 5.0f);

  EXPECT_TRUE(std::isfinite(props.invLocalInertiaTensor[0][0]));
  EXPECT_TRUE(std::isfinite(props.invLocalInertiaTensor[1][1]));
  EXPECT_TRUE(std::isfinite(props.invLocalInertiaTensor[2][2]));
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[0][0], 0.0f);
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[1][1], 0.0f);
  EXPECT_FLOAT_EQ(props.invLocalInertiaTensor[2][2], 0.0f);
}
