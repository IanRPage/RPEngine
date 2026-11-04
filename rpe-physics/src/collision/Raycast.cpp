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

bool raycastSphereAt(Vec3f center, float radius, Vec3f origin, Vec3f dir,
                     float maxDistance, float& distance,
                     Vec3f& normal) noexcept {
  float t0 = 0.0f, t1 = 0.0f;
  if (!sphereInterval(center, radius, origin, dir, t0, t1)) { return false; }
  float t = 0.0f;
  bool exiting = false;
  if (!chooseDistance(t0, t1, maxDistance, t, exiting)) { return false; }
  distance = t;
  normal = normalizedOr(origin + dir * t - center, exiting ? dir : -dir);
  return true;
}

bool raycastCapsuleY(float radius, float halfHeight, Vec3f origin, Vec3f dir,
                     float maxDistance, float& distance,
                     Vec3f& normal) noexcept {
  float tEnter = kInf;
  float tExit = -kInf;
  bool any = false;
  auto merge = [&](float a, float b) {
    tEnter = std::min(tEnter, a);
    tExit = std::max(tExit, b);
    any = true;
  };

  float a = 0.0f, b = 0.0f;
  if (sphereInterval(Vec3f(0.0f, halfHeight, 0.0f), radius, origin, dir, a,
                     b)) {
    merge(a, b);
  }
  if (halfHeight > 0.0f) {
    if (sphereInterval(Vec3f(0.0f, -halfHeight, 0.0f), radius, origin, dir, a,
                       b)) {
      merge(a, b);
    }

    float qa = dir.x * dir.x + dir.z * dir.z;
    float qb = origin.x * dir.x + origin.z * dir.z;
    float qc = origin.x * origin.x + origin.z * origin.z - radius * radius;
    float c0 = -kInf, c1 = kInf;
    bool ok = true;
    if (qa < kZeroLengthEpsilon) {
      ok = qc <= 0.0f;
    } else {
      float disc = qb * qb - qa * qc;
      if (disc < 0.0f) {
        ok = false;
      } else {
        float s = std::sqrt(disc);
        c0 = (-qb - s) / qa;
        c1 = (-qb + s) / qa;
      }
    }
    if (ok) {
      if (std::abs(dir.y) < kZeroLengthEpsilon) {
        ok = std::abs(origin.y) <= halfHeight;
      } else {
        float y0 = (-halfHeight - origin.y) / dir.y;
        float y1 = (halfHeight - origin.y) / dir.y;
        if (y0 > y1) { std::swap(y0, y1); }
        c0 = std::max(c0, y0);
        c1 = std::min(c1, y1);
        ok = c0 <= c1;
      }
    }
    if (ok && std::isfinite(c0) && std::isfinite(c1)) { merge(c0, c1); }
  }

  if (!any) { return false; }

  float t = 0.0f;
  bool exiting = false;
  if (!chooseDistance(tEnter, tExit, maxDistance, t, exiting)) {
    return false;
  }
  Vec3f p = origin + dir * t;
  Vec3f axisPoint(0.0f, std::clamp(p.y, -halfHeight, halfHeight), 0.0f);
  distance = t;
  normal = normalizedOr(p - axisPoint, exiting ? dir : -dir);
  return true;
}

bool raycastPlanes(const Vec4f* planes, std::size_t count, float tolerance,
                   Vec3f origin, Vec3f dir, float maxDistance, float& distance,
                   Vec3f& normal) noexcept {
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

void addUniquePlane(std::vector<Vec4f>& planes, Vec3f n, float w,
                    float tolerance) {
  for (const Vec4f& p : planes) {
    if (glm::dot(Vec3f(p), n) > 1.0f - 1e-5f &&
        std::abs(p.w - w) <= tolerance) {
      return;
    }
  }
  planes.emplace_back(n, w);
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
  if (maxSide <= tolerance) {
    addUniquePlane(planes, n, w, tolerance);
  } else if (minSide >= -tolerance) {
    addUniquePlane(planes, -n, -w, tolerance);
  }
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

bool SphereShape::raycastLocal(Vec3f origin, Vec3f unitDir, float maxDistance,
                               float& distance, Vec3f& normal) const noexcept {
  return raycastSphereAt(Vec3f(0.0f), radius, origin, unitDir, maxDistance,
                         distance, normal);
}

bool BoxShape::raycastLocal(Vec3f origin, Vec3f unitDir, float maxDistance,
                            float& distance, Vec3f& normal) const noexcept {
  const Vec4f planes[6] = {
      Vec4f(1.0f, 0.0f, 0.0f, halfExtents.x),
      Vec4f(-1.0f, 0.0f, 0.0f, halfExtents.x),
      Vec4f(0.0f, 1.0f, 0.0f, halfExtents.y),
      Vec4f(0.0f, -1.0f, 0.0f, halfExtents.y),
      Vec4f(0.0f, 0.0f, 1.0f, halfExtents.z),
      Vec4f(0.0f, 0.0f, -1.0f, halfExtents.z),
  };
  float tolerance = 1e-5f * std::max(1.0f, glm::length(halfExtents));
  return raycastPlanes(planes, 6, tolerance, origin, unitDir, maxDistance,
                       distance, normal);
}

bool CapsuleShape::raycastLocal(Vec3f origin, Vec3f unitDir, float maxDistance,
                                float& distance,
                                Vec3f& normal) const noexcept {
  return raycastCapsuleY(radius, halfHeight, origin, unitDir, maxDistance,
                         distance, normal);
}

std::vector<Vec4f> ConvexHullShape::buildRayPlanes(
    const std::vector<Vec3f>& vertices, float boundingRadius) {
  std::vector<Vec4f> planes;
  if (vertices.empty()) { return planes; }

  const float tol = kHullBuildTolerance * std::max(1.0f, boundingRadius);
  const Vec3f v0 = vertices.front();

  std::size_t i1 = 0;
  float best = 0.0f;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    float d = glm::length(vertices[i] - v0);
    if (d > best) {
      best = d;
      i1 = i;
    }
  }
  if (best <= tol) { return planes; }
  const Vec3f lineDir = (vertices[i1] - v0) / best;

  std::size_t i2 = 0;
  best = 0.0f;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    Vec3f rel = vertices[i] - v0;
    float d = glm::length(rel - lineDir * glm::dot(rel, lineDir));
    if (d > best) {
      best = d;
      i2 = i;
    }
  }
  if (best <= tol) { return planes; }

  const Vec3f planeNormal =
      glm::normalize(glm::cross(vertices[i1] - v0, vertices[i2] - v0));

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

bool ConvexHullShape::raycastLocal(Vec3f origin, Vec3f unitDir,
                                   float maxDistance, float& distance,
                                   Vec3f& normal) const noexcept {
  if (rayPlanes_.empty()) { return false; }
  float tolerance = 1e-5f * std::max(1.0f, cachedBoundingRadius_);
  return raycastPlanes(rayPlanes_.data(), rayPlanes_.size(), tolerance, origin,
                       unitDir, maxDistance, distance, normal);
}

bool CompoundShape::raycastLocal(Vec3f origin, Vec3f unitDir, float maxDistance,
                                 float& distance,
                                 Vec3f& normal) const noexcept {
  Ray localRay{origin, unitDir};
  float bestDistance = maxDistance;
  Vec3f bestNormal(0.0f);
  bool anyHit = false;
  for (const CompoundChild& child : children) {
    RayHit hit =
        raycastShape(child.shape, child.localTransform, localRay, bestDistance);
    if (hit.hit && (!anyHit || hit.distance < bestDistance)) {
      bestDistance = hit.distance;
      bestNormal = hit.normal;
      anyHit = true;
    }
  }
  if (!anyHit) { return false; }
  distance = bestDistance;
  normal = bestNormal;
  return true;
}

RayHit raycastShape(const ShapeVariant& shape, const Transform& transform,
                    const Ray& ray, float maxDistance) noexcept {
  RayHit result;
  if (!(maxDistance >= 0.0f)) { return result; }

  float len = glm::length(ray.direction);
  if (!(len > kZeroLengthEpsilon) || !std::isfinite(len)) { return result; }
  const Vec3f dir = ray.direction / len;

  const Quatf q = glm::normalize(transform.orientation);
  const Quatf invQ = glm::conjugate(q);
  const Vec3f localOrigin = invQ * (ray.origin - transform.position);
  const Vec3f localDir = invQ * dir;

  float distance = 0.0f;
  Vec3f localNormal(0.0f);
  bool hit = std::visit(
      [&](const auto& s) {
        return s.raycastLocal(localOrigin, localDir, maxDistance, distance,
                              localNormal);
      },
      shape);
  if (!hit) { return result; }

  result.hit = true;
  result.distance = distance;
  result.point = ray.origin + dir * distance;
  result.normal = normalizedOr(q * localNormal, -dir);
  return result;
}
