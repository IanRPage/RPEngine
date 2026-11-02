#include <algorithm>
#include <cmath>
#include <collision/Shapes.hpp>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <variant>

#include <math/Constants.hpp>

Mat3f SphereShape::localInertiaTensor(float mass) const noexcept {
  float i = (2.0f / 5.0f) * mass * radius * radius;
  return Mat3f(i, 0.0f, 0.0f, 0.0f, i, 0.0f, 0.0f, 0.0f, i);
}

// textbook box inertia is stated in terms of FULL side lengths:
//   Ixx = m/12 * ((2hy)^2 + (2hz)^2) = m/12 * (4hy^2 + 4hz^2) = m/3 * (hy^2 +
//         hz^2)
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
  const float ixx =
      mc * (h * h / 3.0f + r * r / 4.0f) +
      ms * ((2.0f / 5.0f) * r * r + h * h + (3.0f / 4.0f) * h * r);

  return Mat3f(ixx, 0.0f, 0.0f, 0.0f, iyy, 0.0f, 0.0f, 0.0f, ixx);
}

ConvexHullShape::ConvexHullShape(std::vector<Vec3f> vertices)
    : localVertices(std::move(vertices)) {
  if (localVertices.empty()) {
    throw std::invalid_argument("ConvexHullShape requires at least one vertex");
  }
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
  float area2 = 0.0f;   // 2*A, signed
  float ixArea = 0.0f;  // unscaled sum for integral of y^2 dA
  float iyArea = 0.0f;  // unscaled sum for integral of x^2 dA

  for (std::size_t i = 0; i < n; ++i) {
    const Vec3f& v0 = localVertices[i];
    const Vec3f& v1 = localVertices[(i + 1) % n];
    float cross = v0.x * v1.y - v1.x * v0.y;
    float xTerm = v0.x * v0.x + v0.x * v1.x + v1.x * v1.x;
    float yTerm = v0.y * v0.y + v0.y * v1.y + v1.y * v1.y;
    area2 += cross;
    iyArea += cross * xTerm;
    ixArea += cross * yTerm;
  }

  const float area = 0.5f * std::abs(area2);
  const float ixx = (area > 1e-12f) ? (mass / (12.0f * area)) * ixArea : 0.0f;
  const float iyy = (area > 1e-12f) ? (mass / (12.0f * area)) * iyArea : 0.0f;
  const float izz = ixx + iyy;

  return Mat3f(ixx, 0.0f, 0.0f, 0.0f, iyy, 0.0f, 0.0f, 0.0f, izz);
}

namespace {

AABB unionAABB(const AABB& a, const AABB& b) noexcept {
  return AABB(glm::min(a.min, b.min), glm::max(a.max, b.max));
}

float childBoundingRadius(const ShapeVariant& shape) noexcept {
  return std::visit([](const auto& s) { return s.boundingRadius(); }, shape);
}

AABB transformedCorners(const std::vector<Vec3f>& localPoints,
                        const Transform& t) noexcept {
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

CompoundShape::CompoundShape(std::vector<CompoundChild> childList)
    : children(std::move(childList)) {
  if (children.empty()) {
    throw std::invalid_argument("CompoundShape requires at least one child");
  }
  float total = 0.0f;
  for (const CompoundChild& child : children) {
    if (std::holds_alternative<CompoundShape>(child.shape)) {
      throw std::invalid_argument(
          "CompoundShape children may not themselves be CompoundShapes");
    }
    if (!std::isfinite(child.weight) || !(child.weight > 0.0f)) {
      throw std::invalid_argument(
          "CompoundShape child weight must be finite and positive");
    }
    const Quatf& q = child.localTransform.orientation;
    const Vec3f& p = child.localTransform.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || glm::length(q) < 1e-6f) {
      throw std::invalid_argument(
          "CompoundShape child local transform must be finite and have a "
          "non-degenerate orientation");
    }
    total += child.weight;
  }
  if (!std::isfinite(total) || !(total > 0.0f)) {
    throw std::invalid_argument("CompoundShape total weight must be finite");
  }
  Vec3f weightedPosSum(0.0f);
  for (CompoundChild& child : children) {
    child.localTransform.orientation =
        glm::normalize(child.localTransform.orientation);
    weightedPosSum += child.weight * child.localTransform.position;
  }
  cachedTotalWeight_ = total;
  cachedCenterOfMass_ = weightedPosSum / total;

  float radius = 0.0f;
  AABB box = worldAABB(children.front().shape,
                       children.front().localTransform);
  for (const CompoundChild& child : children) {
    float r = glm::length(child.localTransform.position) +
              childBoundingRadius(child.shape);
    radius = std::max(radius, r);
    box = unionAABB(box, worldAABB(child.shape, child.localTransform));
  }
  cachedBoundingRadius_ = radius;
  cachedLocalAABB_ = box;
}

CompoundShape CompoundShape::recentered() const {
  std::vector<CompoundChild> shifted = children;
  for (CompoundChild& child : shifted) {
    child.localTransform.position -= cachedCenterOfMass_;
  }
  return CompoundShape(std::move(shifted));
}

Vec3f CompoundShape::support(Vec3f direction) const noexcept {
  Vec3f best = worldSupport(children.front().shape,
                            children.front().localTransform, direction);
  float bestDot = glm::dot(best, direction);
  for (std::size_t i = 1; i < children.size(); ++i) {
    Vec3f p = worldSupport(children[i].shape, children[i].localTransform,
                           direction);
    float d = glm::dot(p, direction);
    if (d > bestDot) {
      bestDot = d;
      best = p;
    }
  }
  return best;
}

Mat3f CompoundShape::localInertiaTensor(float mass) const noexcept {
  Mat3f total(0.0f);
  if (!(mass > 0.0f) || !(cachedTotalWeight_ > 0.0f)) { return total; }
  for (const CompoundChild& child : children) {
    const float childMass = mass * (child.weight / cachedTotalWeight_);
    const Mat3f childLocal = std::visit(
        [&](const auto& s) -> Mat3f {
          using ShapeT = std::decay_t<decltype(s)>;
          if constexpr (std::is_same_v<ShapeT, CompoundShape>) {
            return Mat3f(0.0f);
          } else {
            return s.localInertiaTensor(childMass);
          }
        },
        child.shape);
    const Mat3f r = glm::mat3_cast(child.localTransform.orientation);
    Mat3f rotated = r * childLocal * glm::transpose(r);
    const Vec3f d = child.localTransform.position;
    const float d2 = glm::dot(d, d);
    Mat3f shift = Mat3f(d2) - glm::outerProduct(d, d);
    total += rotated + childMass * shift;
  }
  return 0.5f * (total + glm::transpose(total));
}

Mat3f invertInertiaTensor(const Mat3f& tensor) noexcept {
  auto safeInv = [](float v) { return (v > 1e-12f) ? 1.0f / v : 0.0f; };
  const bool diagonal = tensor[0][1] == 0.0f && tensor[0][2] == 0.0f &&
                        tensor[1][0] == 0.0f && tensor[1][2] == 0.0f &&
                        tensor[2][0] == 0.0f && tensor[2][1] == 0.0f;
  if (diagonal) {
    return Mat3f(safeInv(tensor[0][0]), 0.0f, 0.0f, 0.0f,
                 safeInv(tensor[1][1]), 0.0f, 0.0f, 0.0f,
                 safeInv(tensor[2][2]));
  }
  float det = glm::determinant(tensor);
  return (std::abs(det) > 1e-12f) ? glm::inverse(tensor) : Mat3f(0.0f);
}

Vec3f worldSupport(const ShapeVariant& shape, const Transform& t,
                   Vec3f worldDir) noexcept {
  const Vec3f localDir = glm::conjugate(t.orientation) * worldDir;
  const Vec3f localPoint =
      std::visit([&](const auto& s) { return s.support(localDir); }, shape);
  return t.position + (t.orientation * localPoint);
}

AABB worldAABB(const ShapeVariant& shape, const Transform& t) noexcept {
  return std::visit(
      [&](const auto& s) -> AABB {
        using ShapeT = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<ShapeT, SphereShape>) {
          Vec3f r(s.boundingRadius());
          return AABB(t.position - r, t.position + r);
        } else if constexpr (std::is_same_v<ShapeT, CapsuleShape>) {
          Vec3f axisWorld =
              glm::abs(transformDirection(t, Vec3f(0.0f, 1.0f, 0.0f)));
          Vec3f extent = s.halfHeight * axisWorld + Vec3f(s.radius);
          return AABB(t.position - extent, t.position + extent);
        } else if constexpr (std::is_same_v<ShapeT, BoxShape>) {
          Mat3f absR = glm::mat3_cast(t.orientation);
          for (int c = 0; c < 3; ++c) { absR[c] = glm::abs(absR[c]); }
          Vec3f extent = absR * s.halfExtents;
          return AABB(t.position - extent, t.position + extent);
        } else if constexpr (std::is_same_v<ShapeT, ConvexHullShape>) {
          return transformedCorners(s.localVertices, t);
        } else {
          AABB box = worldAABB(s.children.front().shape,
                               compose(t, s.children.front().localTransform));
          for (std::size_t i = 1; i < s.children.size(); ++i) {
            box = unionAABB(box,
                            worldAABB(s.children[i].shape,
                                      compose(t, s.children[i].localTransform)));
          }
          return box;
        }
      },
      shape);
}

bool isFlatPair(const ShapeVariant& shapeA, const Transform& ta,
                const ShapeVariant& shapeB, const Transform& tb) noexcept {
  constexpr float kFlatEpsilon = 1e-5f;
  AABB aabbA = worldAABB(shapeA, ta);
  AABB aabbB = worldAABB(shapeB, tb);
  return (aabbA.max.z - aabbA.min.z) < kFlatEpsilon &&
         (aabbB.max.z - aabbB.min.z) < kFlatEpsilon;
}
