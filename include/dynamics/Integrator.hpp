#ifndef RPENGINE_DYNAMICS_INTEGRATOR_HPP
#define RPENGINE_DYNAMICS_INTEGRATOR_HPP

#include <core/BodyStore.hpp>
#include <math/Types.hpp>

void integrateVelocity(BodyStore& bodies, float dt, Vec3f gravity) noexcept;

void integratePosition(BodyStore& bodies, float dt) noexcept;

class IIntegrator {
 public:
  virtual ~IIntegrator() = default;
  virtual void integrateVelocity(BodyStore& bodies, float dt,
                                 Vec3f gravity) const noexcept = 0;
  virtual void integratePosition(BodyStore& bodies,
                                 float dt) const noexcept = 0;
};

class SemiImplicitEulerIntegrator final : public IIntegrator {
 public:
  void integrateVelocity(BodyStore& bodies, float dt,
                         Vec3f gravity) const noexcept override {
    ::integrateVelocity(bodies, dt, gravity);
  }
  void integratePosition(BodyStore& bodies, float dt) const noexcept override {
    ::integratePosition(bodies, dt);
  }
};

#endif
