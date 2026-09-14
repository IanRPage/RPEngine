#ifndef RPENGINE_MATH_ROTATION_HPP
#define RPENGINE_MATH_ROTATION_HPP

#include <math/Types.hpp>

Quatf integrateOrientation(Quatf q, Vec3f angularVelocity, float dt) noexcept;
Quatf nlerp(Quatf a, Quatf b, float t) noexcept;

#endif
