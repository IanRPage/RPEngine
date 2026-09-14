#include <dynamics/Friction.hpp>

#include <cmath>
#include <math/Constants.hpp>

std::pair<Vec3f, Vec3f> computeTangentBasis(Vec3f normal) noexcept {
  Vec3f tangent1;
  if (std::abs(normal.x) >= INV_SQRT_3) {
    tangent1 = glm::normalize(glm::cross(normal, Vec3f(0.0f, 1.0f, 0.0f)));
  } else {
    tangent1 = glm::normalize(glm::cross(normal, Vec3f(1.0f, 0.0f, 0.0f)));
  }
  Vec3f tangent2 = glm::cross(normal, tangent1);
  return {tangent1, tangent2};
}
