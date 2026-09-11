#pragma once

#include <cmath>
#include <span>

#include <collision/Gjk.hpp>
#include <collision/Epa.hpp>
#include <collision/Manifold.hpp>
#include <collision/MassProperties.hpp>
#include <core/World.hpp>
#include <dynamics/Solver.hpp>

namespace test_helpers {

inline BodyHandle spawnSphere(BodyStore& store, Vec3f pos, float invMass,
                              float radius = 1.0f,
                              Vec3f linearVelocity = Vec3f(0.0f),
                              bool constrainTo2D = false,
                              bool realisticInertia = false) {
  BodyDesc desc;
  desc.shape = ShapeVariant{SphereShape{radius}};
  desc.transform.position = pos;
  desc.invMass = invMass;
  desc.constrainTo2D = constrainTo2D;
  if (realisticInertia && invMass > 0.0f) {
    MassProperties mp = computeMassProperties(desc.shape, 1.0f / invMass);
    desc.invInertiaBody = mp.invLocalInertiaTensor;
  }
  BodyHandle h = store.addBody(desc);
  store.linearVelocity(h) = linearVelocity;
  return h;
}

inline BodyHandle spawnBox(BodyStore& store, Vec3f pos, float invMass,
                           Vec3f halfExtents,
                           Vec3f linearVelocity = Vec3f(0.0f),
                           bool constrainTo2D = false,
                           bool realisticInertia = false) {
  BodyDesc desc;
  desc.shape = ShapeVariant{BoxShape{halfExtents}};
  desc.transform.position = pos;
  desc.invMass = invMass;
  desc.constrainTo2D = constrainTo2D;
  if (realisticInertia && invMass > 0.0f) {
    MassProperties mp = computeMassProperties(desc.shape, 1.0f / invMass);
    desc.invInertiaBody = mp.invLocalInertiaTensor;
  }
  BodyHandle h = store.addBody(desc);
  store.linearVelocity(h) = linearVelocity;
  return h;
}

inline BodyHandle spawnCapsule(BodyStore& store, Vec3f pos, float invMass,
                               float radius, float halfHeight,
                               Vec3f linearVelocity = Vec3f(0.0f),
                               bool constrainTo2D = false,
                               bool realisticInertia = false) {
  BodyDesc desc;
  desc.shape = ShapeVariant{CapsuleShape{radius, halfHeight}};
  desc.transform.position = pos;
  desc.invMass = invMass;
  desc.constrainTo2D = constrainTo2D;
  if (realisticInertia && invMass > 0.0f) {
    MassProperties mp = computeMassProperties(desc.shape, 1.0f / invMass);
    desc.invInertiaBody = mp.invLocalInertiaTensor;
  }
  BodyHandle h = store.addBody(desc);
  store.linearVelocity(h) = linearVelocity;
  return h;
}

inline Manifold buildManifoldBetween(BodyStore& store, BodyHandle a,
                                     BodyHandle b) {
  Transform ta = store.transform(a);
  Transform tb = store.transform(b);
  GjkResult gjk = gjkOverlap(store.shape(a), ta, store.shape(b), tb);
  EpaResult epa = epaPenetration(store.shape(a), ta, store.shape(b), tb, gjk);
  return buildManifold(store.shape(a), ta, store.shape(b), tb, a, b, gjk, epa);
}

inline Quatf rotationAboutAxis(Vec3f axis, float radians) {
  Vec3f n = glm::normalize(axis);
  float half = radians * 0.5f;
  float s = std::sin(half);
  return Quatf(std::cos(half), n.x * s, n.y * s, n.z * s);
}

inline void stepWorld(World& world, int steps, float dt) {
  for (int i = 0; i < steps; i++) { world.step(dt); }
}

inline Vec3f totalLinearMomentum(const BodyStore& store,
                                 std::span<const BodyHandle> handles) {
  Vec3f total(0.0f);
  for (BodyHandle h : handles) {
    float invMass = store.invMass(h);
    if (invMass <= 0.0f) { continue; }
    total += (1.0f / invMass) * store.linearVelocity(h);
  }
  return total;
}

inline Vec3f systemCenterOfMass(const BodyStore& store,
                                std::span<const BodyHandle> handles) {
  Vec3f weightedSum(0.0f);
  float totalMass = 0.0f;
  for (BodyHandle h : handles) {
    float invMass = store.invMass(h);
    if (invMass <= 0.0f) { continue; }
    float mass = 1.0f / invMass;
    weightedSum += mass * store.position(h);
    totalMass += mass;
  }
  return totalMass > 0.0f ? weightedSum / totalMass : Vec3f(0.0f);
}

inline Mat3f inertiaWorld(const BodyStore& store, BodyHandle h) {
  Mat3f r = glm::mat3_cast(store.orientation(h));
  Mat3f invInertiaWorld = r * store.invInertiaBody(h) * glm::transpose(r);
  return glm::inverse(invInertiaWorld);
}

inline Vec3f angularMomentumAboutPoint(const BodyStore& store, BodyHandle h,
                                       Vec3f point) {
  float invMass = store.invMass(h);
  if (invMass <= 0.0f) { return Vec3f(0.0f); }
  float mass = 1.0f / invMass;
  Vec3f r = store.position(h) - point;
  Vec3f orbital = mass * glm::cross(r, store.linearVelocity(h));
  Vec3f spin = inertiaWorld(store, h) * store.angularVelocity(h);
  return orbital + spin;
}

inline Vec3f totalAngularMomentumAboutPoint(const BodyStore& store,
                                            std::span<const BodyHandle> handles,
                                            Vec3f point) {
  Vec3f total(0.0f);
  for (BodyHandle h : handles) {
    if (store.invMass(h) <= 0.0f) { continue; }
    total += angularMomentumAboutPoint(store, h, point);
  }
  return total;
}

inline float kineticEnergy(const BodyStore& store, BodyHandle h) {
  float invMass = store.invMass(h);
  if (invMass <= 0.0f) { return 0.0f; }
  float mass = 1.0f / invMass;
  Vec3f v = store.linearVelocity(h);
  Vec3f w = store.angularVelocity(h);
  float linear = 0.5f * mass * glm::dot(v, v);
  float angular = 0.5f * glm::dot(w, inertiaWorld(store, h) * w);
  return linear + angular;
}

inline float totalKineticEnergy(const BodyStore& store,
                                std::span<const BodyHandle> handles) {
  float total = 0.0f;
  for (BodyHandle h : handles) { total += kineticEnergy(store, h); }
  return total;
}

}  // namespace test_helpers
