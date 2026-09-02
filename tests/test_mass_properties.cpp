#include <gtest/gtest.h>

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
