#include <dynamics/Solver.hpp>

#include <algorithm>
#include <cmath>
#include <dynamics/Friction.hpp>
#include <math/Rotation.hpp>
#include <unordered_map>

namespace {

Mat3f computeInvInertiaWorld(const BodyStore& bodies, BodyHandle h) noexcept {
  Mat3f r = glm::mat3_cast(bodies.orientation(h));
  return r * bodies.invInertiaBody(h) * glm::transpose(r);
}

class InvInertiaWorldCache {
 public:
  explicit InvInertiaWorldCache(const BodyStore& bodies) : bodies_(bodies) {}

  const Mat3f& get(BodyHandle h) noexcept {
    auto [it, inserted] = cache_.try_emplace(h.index);
    if (inserted) { it->second = computeInvInertiaWorld(bodies_, h); }
    return it->second;
  }

 private:
  const BodyStore& bodies_;
  std::unordered_map<uint32_t, Mat3f> cache_;
};

float angularTerm(const Mat3f& invInertiaWorld, Vec3f r, Vec3f axis) noexcept {
  return glm::dot(glm::cross(invInertiaWorld * glm::cross(r, axis), r), axis);
}

}  // namespace

std::vector<float> prepareRestitutionBias(std::span<const Manifold> manifolds,
                                          const BodyStore& bodies) noexcept {
  std::vector<float> bias;
  for (const Manifold& m : manifolds) {
    BodyHandle a = m.bodyA;
    BodyHandle b = m.bodyB;
    Transform ta = bodies.transform(a);
    Transform tb = bodies.transform(b);
    float combinedRestitution =
        std::max(bodies.restitution(a), bodies.restitution(b));

    for (uint8_t i = 0; i < m.pointCount; i++) {
      const ManifoldPoint& point = m.points[i];
      Vec3f rA = transformDirection(ta, point.localAnchorA);
      Vec3f rB = transformDirection(tb, point.localAnchorB);
      Vec3f relVel = (bodies.linearVelocity(b) +
                      glm::cross(bodies.angularVelocity(b), rB)) -
                     (bodies.linearVelocity(a) +
                      glm::cross(bodies.angularVelocity(a), rA));
      float relVelNormal = glm::dot(relVel, m.normal);
      bias.push_back(-combinedRestitution * relVelNormal);
    }
  }
  return bias;
}

void warmStart(std::span<Manifold> manifolds, BodyStore& bodies) noexcept {
  InvInertiaWorldCache invInertiaCache(bodies);

  for (Manifold& m : manifolds) {
    BodyHandle a = m.bodyA;
    BodyHandle b = m.bodyB;
    Transform ta = bodies.transform(a);
    Transform tb = bodies.transform(b);
    const Mat3f& invIA = invInertiaCache.get(a);
    const Mat3f& invIB = invInertiaCache.get(b);
    float invMassA = bodies.invMass(a);
    float invMassB = bodies.invMass(b);
    auto [tangent1, tangent2] = computeTangentBasis(m.normal);

    for (uint8_t i = 0; i < m.pointCount; i++) {
      ManifoldPoint& point = m.points[i];
      Vec3f rA = transformDirection(ta, point.localAnchorA);
      Vec3f rB = transformDirection(tb, point.localAnchorB);

      Vec3f impulse = m.normal * point.normalImpulse +
                      tangent1 * point.tangentImpulse[0] +
                      tangent2 * point.tangentImpulse[1];

      bodies.linearVelocity(a) -= impulse * invMassA;
      bodies.angularVelocity(a) -= invIA * glm::cross(rA, impulse);
      bodies.linearVelocity(b) += impulse * invMassB;
      bodies.angularVelocity(b) += invIB * glm::cross(rB, impulse);
    }
  }
}

void solveVelocity(std::span<Manifold> manifolds, BodyStore& bodies,
                   std::span<const float> restitutionBias) noexcept {
  InvInertiaWorldCache invInertiaCache(bodies);
  size_t biasIndex = 0;

  for (Manifold& m : manifolds) {
    BodyHandle a = m.bodyA;
    BodyHandle b = m.bodyB;
    Transform ta = bodies.transform(a);
    Transform tb = bodies.transform(b);
    const Mat3f& invIA = invInertiaCache.get(a);
    const Mat3f& invIB = invInertiaCache.get(b);
    float invMassA = bodies.invMass(a);
    float invMassB = bodies.invMass(b);
    float invMassSum = invMassA + invMassB;
    // replicates Bullet/Box2D pair-combination rule for friction
    float combinedFriction = std::sqrt(bodies.friction(a) * bodies.friction(b));

    for (uint8_t i = 0; i < m.pointCount; i++) {
      ManifoldPoint& point = m.points[i];
      float targetVelNormal = restitutionBias[biasIndex++];
      Vec3f rA = transformDirection(ta, point.localAnchorA);
      Vec3f rB = transformDirection(tb, point.localAnchorB);

      auto relVel = [&]() noexcept {
        return (bodies.linearVelocity(b) +
                glm::cross(bodies.angularVelocity(b), rB)) -
               (bodies.linearVelocity(a) +
                glm::cross(bodies.angularVelocity(a), rA));
      };

      float angularTermA = angularTerm(invIA, rA, m.normal);
      float angularTermB = angularTerm(invIB, rB, m.normal);
      float denom = invMassSum + angularTermA + angularTermB;
      if (denom > 0.0f) {
        float effectiveMass = 1.0f / denom;
        float relVelNormal = glm::dot(relVel(), m.normal);
        float deltaImpulse = effectiveMass * (targetVelNormal - relVelNormal);

        float newAccumulated =
            std::max(point.normalImpulse + deltaImpulse, 0.0f);
        deltaImpulse = newAccumulated - point.normalImpulse;
        point.normalImpulse = newAccumulated;

        Vec3f impulseVec = m.normal * deltaImpulse;
        bodies.linearVelocity(a) -= impulseVec * invMassA;
        bodies.angularVelocity(a) -= invIA * glm::cross(rA, impulseVec);
        bodies.linearVelocity(b) += impulseVec * invMassB;
        bodies.angularVelocity(b) += invIB * glm::cross(rB, impulseVec);
      }

      // uses a simplified friction cone for box friction model
      auto [tangent1, tangent2] = computeTangentBasis(m.normal);
      Vec3f tangents[2] = {tangent1, tangent2};
      for (int t = 0; t < 2; t++) {
        Vec3f tangent = tangents[t];
        float angularTermTA = angularTerm(invIA, rA, tangent);
        float angularTermTB = angularTerm(invIB, rB, tangent);
        float denomT = invMassSum + angularTermTA + angularTermTB;
        if (denomT <= 0.0f) { continue; }

        float effectiveMassT = 1.0f / denomT;
        float relVelTangent = glm::dot(relVel(), tangent);
        float deltaTangentImpulse = -effectiveMassT * relVelTangent;

        float maxFriction = combinedFriction * point.normalImpulse;
        float newTangentImpulse =
            std::clamp(point.tangentImpulse[t] + deltaTangentImpulse,
                       -maxFriction, maxFriction);
        deltaTangentImpulse = newTangentImpulse - point.tangentImpulse[t];
        point.tangentImpulse[t] = newTangentImpulse;

        Vec3f impulseVec = tangent * deltaTangentImpulse;
        bodies.linearVelocity(a) -= impulseVec * invMassA;
        bodies.angularVelocity(a) -= invIA * glm::cross(rA, impulseVec);
        bodies.linearVelocity(b) += impulseVec * invMassB;
        bodies.angularVelocity(b) += invIB * glm::cross(rB, impulseVec);
      }
    }
  }
}

void solvePosition(std::span<Manifold> manifolds, BodyStore& bodies,
                   const SolverConfig& config) noexcept {
  for (Manifold& m : manifolds) {
    BodyHandle a = m.bodyA;
    BodyHandle b = m.bodyB;
    float invMassA = bodies.invMass(a);
    float invMassB = bodies.invMass(b);

    for (uint8_t i = 0; i < m.pointCount; i++) {
      const ManifoldPoint& point = m.points[i];

      Transform ta = bodies.transform(a);
      Transform tb = bodies.transform(b);
      Vec3f worldAnchorA = transformPoint(ta, point.localAnchorA);
      Vec3f worldAnchorB = transformPoint(tb, point.localAnchorB);
      float currentPenetration =
          glm::dot(worldAnchorB - worldAnchorA, m.normal);
      if (currentPenetration <= config.slop) { continue; }

      Vec3f rA = transformDirection(ta, point.localAnchorA);
      Vec3f rB = transformDirection(tb, point.localAnchorB);
      Mat3f invIA = computeInvInertiaWorld(bodies, a);
      Mat3f invIB = computeInvInertiaWorld(bodies, b);
      float angularTermA = angularTerm(invIA, rA, m.normal);
      float angularTermB = angularTerm(invIB, rB, m.normal);
      float denom = invMassA + invMassB + angularTermA + angularTermB;
      if (denom <= 0.0f) { continue; }

      float correctionMagnitude = std::min(
          config.baumgartePositionFactor * (currentPenetration - config.slop),
          config.maxCorrectionPerIteration);
      Vec3f correction = m.normal * (correctionMagnitude / denom);

      bodies.position(a) -= correction * invMassA;
      bodies.position(b) += correction * invMassB;

      bodies.orientation(a) = integrateOrientation(
          bodies.orientation(a), -(invIA * glm::cross(rA, correction)), 1.0f);
      bodies.orientation(b) = integrateOrientation(
          bodies.orientation(b), invIB * glm::cross(rB, correction), 1.0f);
    }
  }
}
