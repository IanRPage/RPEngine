#include <collision/MassProperties.hpp>

MassProperties computeMassProperties(const ShapeVariant& shape,
                                     float mass) noexcept {
  MassProperties props;
  props.mass = mass;
  props.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
  props.localInertiaTensor = std::visit(
      [&](const auto& s) { return s.localInertiaTensor(mass); }, shape);
  props.invLocalInertiaTensor =
      (mass <= 0.0f) ? Mat3f(0.0f)
                     : invertInertiaTensor(props.localInertiaTensor);
  return props;
}
