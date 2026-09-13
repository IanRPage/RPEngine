#ifndef RPENGINE_DYNAMICS_FRICTION_HPP
#define RPENGINE_DYNAMICS_FRICTION_HPP

#include <math/Types.hpp>
#include <utility>

std::pair<Vec3f, Vec3f> computeTangentBasis(Vec3f normal) noexcept;

#endif
