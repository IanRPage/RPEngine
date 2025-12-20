#ifndef ENUMS_HPP
#define ENUMS_HPP

constexpr float MIN_PARTICLE_SIZE = 1.0f;
constexpr float MIN_PARTICLE_MASS = 0.01f;

enum class IntegrationType { Euler, Verlet };
enum class BroadphaseType { Naive, Qtree, SpatialGrid };
enum class ForceType { None, Radial };
enum class SpawnType { None, Manual, Random, Stream, Max };

#endif
