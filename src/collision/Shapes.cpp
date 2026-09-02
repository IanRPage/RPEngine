#include <collision/Shapes.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <math/Constants.hpp>

Mat3f SphereShape::localInertiaTensor(float mass) const noexcept {
  float i = (2.0f / 5.0f) * mass * radius * radius;
  return Mat3f(i, 0.0f, 0.0f, 0.0f, i, 0.0f, 0.0f, 0.0f, i);
}

// textbook box inertia is stated in terms of FULL side lengths:
//   Ixx = m/12 * ((2hy)^2 + (2hz)^2) = m/12 * (4hy^2 + 4hz^2) = m/3 * (hy^2 + hz^2)
Mat3f BoxShape::localInertiaTensor(float mass) const noexcept {
  float hx = halfExtents.x, hy = halfExtents.y, hz = halfExtents.z;
  float ixx = (mass / 3.0f) * (hy * hy + hz * hz);
  float iyy = (mass / 3.0f) * (hx * hx + hz * hz);
  float izz = (mass / 3.0f) * (hx * hx + hy * hy);
  return Mat3f(ixx, 0.0f, 0.0f, 0.0f, iyy, 0.0f, 0.0f, 0.0f, izz);
}

// composite of cylindrical body plus two hemispherical caps (the two
// hemispheres together forming one full sphere), mass-split by volume.
// Reference: Millington, "Game Physics Engine Development" capsule inertia
// derivation; checked against Bullet's btCapsuleShape::calculateLocalInertia
Mat3f CapsuleShape::localInertiaTensor(float mass) const noexcept {
  const float r = radius;
  const float h = halfHeight;

  const float vCyl = PI * r * r * (2.0f * h);
  const float vCaps = (4.0f / 3.0f) * PI * r * r * r;
  const float mc = mass * vCyl / (vCyl + vCaps);
  const float ms = mass - mc;

  // axial (about local Y). both pieces already centered on this axis, so no
  // parallel-axis shift needed here
  const float iyy = mc * (r * r / 2.0f) + ms * (2.0f / 5.0f) * (r * r);

  // transverse (Ixx == Izz, about an axis through centroid perpendicular to Y).
  // combined cylinder-plus-offset-hemispheres term, parallel-axis shift folded
  // into formula
  const float ixx = mc * (h * h / 3.0f + r * r / 4.0f) +
                     ms * ((2.0f / 5.0f) * r * r + h * h + (3.0f / 8.0f) * h * r);

  return Mat3f(ixx, 0.0f, 0.0f, 0.0f, iyy, 0.0f, 0.0f, 0.0f, ixx);
}

ConvexHullShape::ConvexHullShape(std::vector<Vec3f> vertices)
    : localVertices(std::move(vertices)) {
  float maxDistSq = 0.0f;
  for (const Vec3f& v : localVertices) {
    maxDistSq = std::max(maxDistSq, glm::dot(v, v));
  }
  cachedBoundingRadius_ = std::sqrt(maxDistSq);
}

Vec3f ConvexHullShape::support(Vec3f direction) const noexcept {
  Vec3f best = localVertices[0];
  float bestDot = glm::dot(best, direction);
  for (std::size_t i = 1; i < localVertices.size(); ++i) {
    float d = glm::dot(localVertices[i], direction);
    if (d > bestDot) {
      bestDot = d;
      best = localVertices[i];
    }
  }
  return best;
}

// TODO(3D hull inertia): tetrahedron decomposition needed once real 3D hull
// shapes exist
Mat3f ConvexHullShape::localInertiaTensor(float mass) const noexcept {
  const std::size_t n = localVertices.size();
  float area2 = 0.0f;     // 2*A, signed
  float numerator = 0.0f;

  for (std::size_t i = 0; i < n; ++i) {
    const Vec3f& v0 = localVertices[i];
    const Vec3f& v1 = localVertices[(i + 1) % n];
    float cross = v0.x * v1.y - v1.x * v0.y;
    float dotTerm = v0.x * v0.x + v0.y * v0.y + v0.x * v1.x + v0.y * v1.y +
                     v1.x * v1.x + v1.y * v1.y;
    area2 += cross;
    numerator += cross * dotTerm;
  }

  const float area = 0.5f * std::abs(area2);
  const float izz = (area > 1e-12f) ? (mass / (6.0f * area)) * numerator : 0.0f;
  const float ixx = izz / 2.0f;

  return Mat3f(ixx, 0.0f, 0.0f, 0.0f, ixx, 0.0f, 0.0f, 0.0f, izz);
}

namespace {

AABB transformedCorners(const std::vector<Vec3f>& localPoints, const Transform& t) noexcept {
  Vec3f worldMin(std::numeric_limits<float>::max());
  Vec3f worldMax(std::numeric_limits<float>::lowest());
  for (const Vec3f& p : localPoints) {
    Vec3f worldPoint = transformPoint(t, p);
    worldMin = glm::min(worldMin, worldPoint);
    worldMax = glm::max(worldMax, worldPoint);
  }
  return AABB(worldMin, worldMax);
}

}  // namespace

Vec3f worldSupport(const ShapeVariant& shape, const Transform& t, Vec3f worldDir) noexcept {
  const Vec3f localDir = glm::inverse(t.orientation) * worldDir;
  const Vec3f localPoint = std::visit([&](const auto& s) { return s.support(localDir); }, shape);
  return t.position + (t.orientation * localPoint);
}

AABB worldAABB(const ShapeVariant& shape, const Transform& t) noexcept {
  return std::visit(
      [&](const auto& s) -> AABB {
        using ShapeT = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<ShapeT, SphereShape> || std::is_same_v<ShapeT, CapsuleShape>) {
          Vec3f r(s.boundingRadius());
          return AABB(t.position - r, t.position + r);
        } else if constexpr (std::is_same_v<ShapeT, BoxShape>) {
          std::vector<Vec3f> corners;
          corners.reserve(8);
          for (float sx : {-1.0f, 1.0f})
            for (float sy : {-1.0f, 1.0f})
              for (float sz : {-1.0f, 1.0f})
                corners.emplace_back(sx * s.halfExtents.x, sy * s.halfExtents.y, sz * s.halfExtents.z);
          return transformedCorners(corners, t);
        } else {  // ConvexHullShape
          return transformedCorners(s.localVertices, t);
        }
      },
      shape);
}
