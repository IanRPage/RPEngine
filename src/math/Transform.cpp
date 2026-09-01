#include <math/Transform.hpp>

Vec3f transformPoint(const Transform& t, Vec3f localPoint) noexcept {
  return t.position + (t.orientation * localPoint);
}

Vec3f transformDirection(const Transform& t, Vec3f localDir) noexcept {
  return t.orientation * localDir;
}

Transform inverse(const Transform& t) noexcept {
  Quatf invOrientation = glm::conjugate(t.orientation);  // conjugate = inverse for unit quaternions
  Transform result;
  result.orientation = invOrientation;
  result.position = invOrientation * (-t.position);
  return result;
}

Transform compose(const Transform& parent, const Transform& child) noexcept {
  Transform result;
  result.orientation = parent.orientation * child.orientation;
  result.position = transformPoint(parent, child.position);
  return result;
}
