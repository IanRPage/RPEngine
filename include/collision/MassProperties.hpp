#ifndef RPENGINE_COLLISION_MASSPROPERTIES_HPP
#define RPENGINE_COLLISION_MASSPROPERTIES_HPP

#include <collision/Shapes.hpp>
#include <math/Types.hpp>

struct MassProperties {
  float mass;
  float invMass;
  Mat3f localInertiaTensor;
  Mat3f invLocalInertiaTensor;
};

MassProperties computeMassProperties(const ShapeVariant& shape, float mass) noexcept;

#endif
