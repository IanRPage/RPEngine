#include <collision/MassProperties.hpp>

namespace {

Mat3f invertDiagonal(const Mat3f& diag, bool isStatic) noexcept {
  if (isStatic) return Mat3f(0.0f);
  return Mat3f(1.0f / diag[0][0], 0.0f, 0.0f, 0.0f, 1.0f / diag[1][1], 0.0f, 0.0f, 0.0f,
               1.0f / diag[2][2]);
}

}  // namespace

MassProperties computeMassProperties(const ShapeVariant& shape, float mass) noexcept {
  MassProperties props;
  props.mass = mass;
  props.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
  props.localInertiaTensor =
      std::visit([&](const auto& s) { return s.localInertiaTensor(mass); }, shape);
  props.invLocalInertiaTensor = invertDiagonal(props.localInertiaTensor, mass <= 0.0f);
  return props;
}
