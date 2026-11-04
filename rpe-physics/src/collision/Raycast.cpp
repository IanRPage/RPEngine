#include <collision/Raycast.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kParallelEpsilon = 1e-7f;
constexpr float kZeroLengthEpsilon = 1e-12f;
constexpr float kHullBuildTolerance = 1e-4f;

bool chooseDistance(float tEnter, float tExit, float maxDistance, float& t,
                    bool& exiting) noexcept {
  if (!(tEnter <= tExit) || tExit < 0.0f) { return false; }
  exiting = tEnter < 0.0f;
  t = exiting ? tExit : tEnter;
  return std::isfinite(t) && t <= maxDistance;
}

Vec3f normalizedOr(Vec3f v, Vec3f fallback) noexcept {
  float len = glm::length(v);
  return len > kZeroLengthEpsilon ? v / len : fallback;
}

bool sphereInterval(Vec3f center, float radius, Vec3f origin, Vec3f dir,
                    float& t0, float& t1) noexcept {
  Vec3f m = origin - center;
  float b = glm::dot(m, dir);
  float c = glm::dot(m, m) - radius * radius;
  float disc = b * b - c;
  if (disc < 0.0f) { return false; }
  float s = std::sqrt(disc);
  t0 = -b - s;
  t1 = -b + s;
  return true;
}

bool raycastPlanes(const Vec4f* planes, std::size_t count, float boundingRadius,
                   Vec3f origin, Vec3f dir, float maxDistance, float& distance,
                   Vec3f& normal) noexcept {
  const float tolerance = 1e-5f * std::max(1.0f, boundingRadius);
  float tEnter = -kInf;
  float tExit = kInf;
  Vec3f nEnter(0.0f), nExit(0.0f);

  for (std::size_t i = 0; i < count; ++i) {
    Vec3f n(planes[i]);
    float s = glm::dot(n, origin) - planes[i].w;
    float denom = glm::dot(n, dir);
    if (std::abs(denom) < kParallelEpsilon) {
      if (s > tolerance) { return false; }
      continue;
    }
    float t = -s / denom;
    if (denom < 0.0f) {
      if (t > tEnter) {
        tEnter = t;
        nEnter = n;
      }
    } else if (t < tExit) {
      tExit = t;
      nExit = n;
    }
    if (tEnter > tExit) { return false; }
  }

  if (!std::isfinite(tEnter) || !std::isfinite(tExit)) { return false; }

  float t = 0.0f;
  bool exiting = false;
  if (!chooseDistance(tEnter, tExit, maxDistance, t, exiting)) {
    return false;
  }
  distance = t;
  normal = exiting ? nExit : nEnter;
  return true;
}

void trySupportingPlane(std::vector<Vec4f>& planes,
                        const std::vector<Vec3f>& vertices, Vec3f n,
                        Vec3f point, float tolerance) {
  float w = glm::dot(n, point);
  float maxSide = -kInf;
  float minSide = kInf;
  for (const Vec3f& v : vertices) {
    float side = glm::dot(n, v) - w;
    maxSide = std::max(maxSide, side);
    minSide = std::min(minSide, side);
  }
  if (maxSide > tolerance) {
    if (minSide < -tolerance) { return; }
    n = -n;
    w = -w;
  }
  for (const Vec4f& p : planes) {
    if (glm::dot(Vec3f(p), n) > 1.0f - 1e-5f &&
        std::abs(p.w - w) <= tolerance) {
      return;
    }
  }
  planes.emplace_back(n, w);
}

std::vector<Vec4f> buildRayPlanes(const std::vector<Vec3f>& vertices,
                                  float boundingRadius) {
  std::vector<Vec4f> planes;
  if (vertices.empty()) { return planes; }

  const float tol = kHullBuildTolerance * std::max(1.0f, boundingRadius);
  const Vec3f v0 = vertices.front();

  Vec3f lineDir(0.0f);
  float best = 0.0f;
  for (const Vec3f& v : vertices) {
    float d = glm::length(v - v0);
    if (d > best) {
      best = d;
      lineDir = (v - v0) / d;
    }
  }
  if (best <= tol) { return planes; }

  Vec3f planeNormal(0.0f);
  best = 0.0f;
  for (const Vec3f& v : vertices) {
    Vec3f c = glm::cross(lineDir, v - v0);
    float d = glm::length(c);
    if (d > best) {
      best = d;
      planeNormal = c / d;
    }
  }
  if (best <= tol) { return planes; }

  float maxOffPlane = 0.0f;
  for (const Vec3f& v : vertices) {
    maxOffPlane = std::max(maxOffPlane, std::abs(glm::dot(v - v0, planeNormal)));
  }

  const std::size_t n = vertices.size();

  if (maxOffPlane <= tol) {
    float w = glm::dot(planeNormal, v0);
    planes.emplace_back(planeNormal, w);
    planes.emplace_back(-planeNormal, -w);
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        Vec3f edge = vertices[j] - vertices[i];
        if (glm::length(edge) <= tol) { continue; }
        Vec3f m = glm::normalize(glm::cross(edge, planeNormal));
        trySupportingPlane(planes, vertices, m, vertices[i], tol);
      }
    }
    return planes;
  }

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      for (std::size_t k = j + 1; k < n; ++k) {
        Vec3f c = glm::cross(vertices[j] - vertices[i],
                             vertices[k] - vertices[i]);
        float len = glm::length(c);
        if (len <= tol * tol) { continue; }
        trySupportingPlane(planes, vertices, c / len, vertices[i], tol);
      }
    }
  }
  return planes;
}

bool raycastTransformed(const ShapeVariant& shape, const Transform& transform,
                        Vec3f origin, Vec3f unitDir, float maxDistance,
                        float& distance, Vec3f& normal) noexcept;

bool raycastLocal(const SphereShape& s, Vec3f origin, Vec3f unitDir,
                  float maxDistance, float& distance, Vec3f& normal) noexcept {
  float t0 = 0.0f, t1 = 0.0f;
  if (!sphereInterval(Vec3f(0.0f), s.radius, origin, unitDir, t0, t1)) {
    return false;
  }
  float t = 0.0f;
  bool exiting = false;
  if (!chooseDistance(t0, t1, maxDistance, t, exiting)) { return false; }
  distance = t;
  normal = normalizedOr(origin + unitDir * t, exiting ? unitDir : -unitDir);
  return true;
}

bool raycastLocal(const BoxShape& s, Vec3f origin, Vec3f unitDir,
                  float maxDistance, float& distance, Vec3f& normal) noexcept {
  const Vec4f planes[6] = {
      Vec4f(1.0f, 0.0f, 0.0f, s.halfExtents.x),
      Vec4f(-1.0f, 0.0f, 0.0f, s.halfExtents.x),
      Vec4f(0.0f, 1.0f, 0.0f, s.halfExtents.y),
      Vec4f(0.0f, -1.0f, 0.0f, s.halfExtents.y),
      Vec4f(0.0f, 0.0f, 1.0f, s.halfExtents.z),
      Vec4f(0.0f, 0.0f, -1.0f, s.halfExtents.z),
  };
  return raycastPlanes(planes, 6, s.boundingRadius(), origin, unitDir,
                       maxDistance, distance, normal);
}

bool raycastLocal(const CapsuleShape& s, Vec3f origin, Vec3f unitDir,
                  float maxDistance, float& distance, Vec3f& normal) noexcept {
  float tEnter = kInf;
  float tExit = -kInf;
  bool any = false;
  auto merge = [&](float a, float b) {
    tEnter = std::min(tEnter, a);
    tExit = std::max(tExit, b);
    any = true;
  };

  float a = 0.0f, b = 0.0f;
  if (sphereInterval(Vec3f(0.0f, s.halfHeight, 0.0f), s.radius, origin, unitDir,
                     a, b)) {
    merge(a, b);
  }
  if (s.halfHeight > 0.0f) {
    if (sphereInterval(Vec3f(0.0f, -s.halfHeight, 0.0f), s.radius, origin,
                       unitDir, a, b)) {
      merge(a, b);
    }

    float qa = unitDir.x * unitDir.x + unitDir.z * unitDir.z;
    float qb = origin.x * unitDir.x + origin.z * unitDir.z;
    float qc = origin.x * origin.x + origin.z * origin.z - s.radius * s.radius;
    float disc = qb * qb - qa * qc;
    if (qa >= kZeroLengthEpsilon && disc >= 0.0f) {
      float root = std::sqrt(disc);
      float c0 = (-qb - root) / qa;
      float c1 = (-qb + root) / qa;
      if (std::abs(unitDir.y) >= kZeroLengthEpsilon) {
        float y0 = (-s.halfHeight - origin.y) / unitDir.y;
        float y1 = (s.halfHeight - origin.y) / unitDir.y;
        if (y0 > y1) { std::swap(y0, y1); }
        c0 = std::max(c0, y0);
        c1 = std::min(c1, y1);
      } else if (std::abs(origin.y) > s.halfHeight) {
        c0 = 1.0f;
        c1 = 0.0f;
      }
      if (c0 <= c1) { merge(c0, c1); }
    }
  }

  if (!any) { return false; }

  float t = 0.0f;
  bool exiting = false;
  if (!chooseDistance(tEnter, tExit, maxDistance, t, exiting)) {
    return false;
  }
  Vec3f p = origin + unitDir * t;
  Vec3f axisPoint(0.0f, std::clamp(p.y, -s.halfHeight, s.halfHeight), 0.0f);
  distance = t;
  normal = normalizedOr(p - axisPoint, exiting ? unitDir : -unitDir);
  return true;
}

bool raycastLocal(const ConvexHullShape& s, Vec3f origin, Vec3f unitDir,
                  float maxDistance, float& distance, Vec3f& normal) noexcept {
  const float radius = s.boundingRadius();
  std::vector<Vec4f> planes = buildRayPlanes(s.localVertices, radius);
  if (planes.empty()) { return false; }
  return raycastPlanes(planes.data(), planes.size(), radius, origin, unitDir,
                       maxDistance, distance, normal);
}

bool raycastLocal(const CompoundShape& s, Vec3f origin, Vec3f unitDir,
                  float maxDistance, float& distance, Vec3f& normal) noexcept {
  float bestDistance = maxDistance;
  Vec3f bestNormal(0.0f);
  bool anyHit = false;
  for (const CompoundChild& child : s.children) {
    float childDist = 0.0f;
    Vec3f childNormal(0.0f);
    if (raycastTransformed(child.shape, child.localTransform, origin, unitDir,
                           bestDistance, childDist, childNormal) &&
        (!anyHit || childDist < bestDistance)) {
      bestDistance = childDist;
      bestNormal = childNormal;
      anyHit = true;
    }
  }
  if (!anyHit) { return false; }
  distance = bestDistance;
  normal = bestNormal;
  return true;
}

bool raycastTransformed(const ShapeVariant& shape, const Transform& transform,
                        Vec3f origin, Vec3f unitDir, float maxDistance,
                        float& distance, Vec3f& normal) noexcept {
  const Quatf q = glm::normalize(transform.orientation);
  const Quatf invQ = glm::conjugate(q);
  const Vec3f localOrigin = invQ * (origin - transform.position);
  const Vec3f localDir = invQ * unitDir;

  Vec3f localNormal(0.0f);
  bool hit = std::visit(
      [&](const auto& s) {
        return raycastLocal(s, localOrigin, localDir, maxDistance, distance,
                            localNormal);
      },
      shape);
  if (!hit) { return false; }

  normal = normalizedOr(q * localNormal, -unitDir);
  return true;
}

}  // namespace

bool rayIntersectsAABB(const Ray& ray, const AABB& aabb, float maxDistance,
                       float* tEnter, float* tExit) noexcept {
  float t0 = 0.0f;
  float t1 = maxDistance;
  if (!(t1 >= 0.0f)) { return false; }
  for (int axis = 0; axis < 3; ++axis) {
    float o = ray.origin[axis];
    float d = ray.direction[axis];
    float mn = aabb.min[axis];
    float mx = aabb.max[axis];
    if (std::abs(d) < kZeroLengthEpsilon) {
      if (o < mn || o > mx) { return false; }
      continue;
    }
    float inv = 1.0f / d;
    float a = (mn - o) * inv;
    float b = (mx - o) * inv;
    if (a > b) { std::swap(a, b); }
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    if (t0 > t1) { return false; }
  }
  if (tEnter != nullptr) { *tEnter = t0; }
  if (tExit != nullptr) { *tExit = t1; }
  return true;
}

RayHit raycastShape(const ShapeVariant& shape, const Transform& transform,
                    const Ray& ray, float maxDistance) noexcept {
  RayHit result;
  if (!(maxDistance >= 0.0f)) { return result; }

  float len = glm::length(ray.direction);
  if (!(len > kZeroLengthEpsilon) || !std::isfinite(len)) { return result; }
  const Vec3f unitDir = ray.direction / len;

  if (!raycastTransformed(shape, transform, ray.origin, unitDir, maxDistance,
                          result.distance, result.normal)) {
    return result;
  }

  result.hit = true;
  result.point = ray.origin + unitDir * result.distance;
  return result;
}
