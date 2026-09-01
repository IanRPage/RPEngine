#include <math/Rotation.hpp>

Quatf integrateOrientation(Quatf q, Vec3f angularVelocity, float dt) noexcept {
  Quatf omega{0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z};
  Quatf dq = omega * q;
  Quatf qNext = q + 0.5f * dt * dq;  // dq -> d(q)/dt = 0.5 * omega * q
  return glm::normalize(qNext);
}

Quatf nlerp(Quatf a, Quatf b, float t) noexcept {
  if (glm::dot(a, b) < 0.0f) {
    b = -b;  // take shorter path around hypersphere
  }
  return glm::normalize(glm::mix(a, b, t));
}
