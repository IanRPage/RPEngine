#include <dynamics/Integrator.hpp>

#include <math/Constraints2D.hpp>
#include <math/Rotation.hpp>

void integrateVelocity(BodyStore& bodies, float dt, Vec3f gravity) noexcept {
  Vec3f deltaVelocity = gravity * dt;
  for (BodyHandle h : bodies.liveHandles()) {
    if (bodies.invMass(h) <= 0.0f) { continue; }
    bodies.linearVelocity(h) += deltaVelocity;
  }
}

void integratePosition(BodyStore& bodies, float dt) noexcept {
  for (BodyHandle h : bodies.liveHandles()) {
    if (bodies.invMass(h) <= 0.0f) { continue; }

    bodies.position(h) += bodies.linearVelocity(h) * dt;
    bodies.orientation(h) = integrateOrientation(bodies.orientation(h),
                                                 bodies.angularVelocity(h), dt);

    if (bodies.constrainTo2D(h)) {
      apply2DConstraint(bodies.position(h), bodies.linearVelocity(h),
                        bodies.angularVelocity(h));
    }
  }
}
