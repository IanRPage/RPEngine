#include <math/Constraints2D.hpp>

void apply2DConstraint(Vec3f& position, Vec3f& linearVelocity,
                        Vec3f& angularVelocity) noexcept {
  position.z = 0.0f;
  linearVelocity.z = 0.0f;
  angularVelocity.x = 0.0f;
  angularVelocity.y = 0.0f;
  // angularVelocity.z // the one rotational DoF a 2D body keeps
}
