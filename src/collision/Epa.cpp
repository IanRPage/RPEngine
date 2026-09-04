#include <collision/Epa.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <math/Constants.hpp>
#include <vector>

namespace {

constexpr float kEpaEpsilon = 1e-4f;
constexpr int kMaxIterations = 32;

Vec3f barycentricLerp(const Vec3f& a, const Vec3f& b, float t) noexcept {
  return a + t * (b - a);
}

void triangleBarycentric(Vec3f a, Vec3f b, Vec3f c, Vec3f p, float& u,
                         float& v, float& w) noexcept {
  Vec3f v0 = b - a, v1 = c - a, v2 = p - a;
  float d00 = glm::dot(v0, v0);
  float d01 = glm::dot(v0, v1);
  float d11 = glm::dot(v1, v1);
  float d20 = glm::dot(v2, v0);
  float d21 = glm::dot(v2, v1);
  float denom = d00 * d11 - d01 * d01;
  if (std::abs(denom) < VECTOR_LENGTH_EPSILON) {
    u = 1.0f;
    v = 0.0f;
    w = 0.0f;
    return;
  }
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;
}

bool isSphere(const ShapeVariant& shape) noexcept {
  return std::holds_alternative<SphereShape>(shape);
}

EpaResult sphereVsSphere(const SphereShape& a, const Transform& ta,
                         const SphereShape& b, const Transform& tb) noexcept {
  Vec3f delta = tb.position - ta.position;
  float dist = glm::length(delta);
  Vec3f normal = (dist > VECTOR_LENGTH_EPSILON) ? (delta / dist)
                                                : Vec3f(1.0f, 0.0f, 0.0f);
  EpaResult result;
  result.normal = normal;
  result.penetrationDepth = a.radius + b.radius - dist;
  result.contactPointOnA = ta.position + normal * a.radius;
  result.contactPointOnB = tb.position - normal * b.radius;
  return result;
}

// ---- 2D EPA: expanding polygon of edge-insertion points ----

struct ClosestEdge {
  std::size_t i0, i1;
  Vec3f normal;
  float distance;
};

float signedArea2D(const std::vector<SupportPoint>& poly) noexcept {
  float area = 0.0f;
  std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t j = (i + 1) % n;
    area += poly[i].diff.x * poly[j].diff.y - poly[j].diff.x * poly[i].diff.y;
  }
  return 0.5f * area;
}

ClosestEdge findClosestEdge2D(const std::vector<SupportPoint>& poly) noexcept {
  ClosestEdge best{0, 0, Vec3f(0.0f), std::numeric_limits<float>::max()};
  std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t j = (i + 1) % n;
    Vec3f e = poly[j].diff - poly[i].diff;
    float len = glm::length(e);
    if (len < VECTOR_LENGTH_EPSILON) continue;
    Vec3f normal(e.y / len, -e.x / len, 0.0f);  // CCW-winding outward normal
    float distance = glm::dot(normal, poly[i].diff);
    if (distance < best.distance) best = {i, j, normal, distance};
  }
  return best;
}

EpaResult epa2D(const ShapeVariant& shapeA, const Transform& ta,
               const ShapeVariant& shapeB, const Transform& tb,
               const GjkResult& terminalSimplex) noexcept {
  std::vector<SupportPoint> poly(
      terminalSimplex.simplex.begin(),
      terminalSimplex.simplex.begin() + terminalSimplex.simplexCount);
  if (signedArea2D(poly) < 0.0f) std::reverse(poly.begin(), poly.end());

  ClosestEdge edge = findClosestEdge2D(poly);
  for (int iter = 0; iter < kMaxIterations; ++iter) {
    SupportPoint support =
        minkowskiSupport(shapeA, ta, shapeB, tb, edge.normal);
    float newDistance = glm::dot(edge.normal, support.diff);

    if (newDistance - edge.distance < kEpaEpsilon) break;

    poly.insert(poly.begin() + static_cast<long>(edge.i1), support);
    edge = findClosestEdge2D(poly);
  }

  const SupportPoint& a = poly[edge.i0];
  const SupportPoint& b = poly[edge.i1];
  Vec3f closestPoint = edge.normal * edge.distance;
  Vec3f ab = b.diff - a.diff;
  float denom = glm::dot(ab, ab);
  float t = (denom > VECTOR_LENGTH_EPSILON)
                ? glm::dot(closestPoint - a.diff, ab) / denom
                : 0.0f;
  t = glm::clamp(t, 0.0f, 1.0f);

  EpaResult result;
  result.normal = edge.normal;
  result.penetrationDepth = edge.distance;
  result.contactPointOnA = barycentricLerp(a.a, b.a, t);
  result.contactPointOnB = barycentricLerp(a.b, b.b, t);
  return result;
}

// ---- 3D EPA: expanding polytope of triangular faces ----

struct Face {
  int a, b, c;
  Vec3f normal;
  float distance;
};

Face makeFace(const std::vector<SupportPoint>& verts, int ia, int ib, int ic,
             Vec3f centroid) noexcept {
  Vec3f A = verts[static_cast<std::size_t>(ia)].diff;
  Vec3f B = verts[static_cast<std::size_t>(ib)].diff;
  Vec3f C = verts[static_cast<std::size_t>(ic)].diff;
  Vec3f normal = glm::cross(B - A, C - A);
  float len = glm::length(normal);
  if (len > VECTOR_LENGTH_EPSILON) normal /= len;
  if (glm::dot(normal, A - centroid) < 0.0f) normal = -normal;
  return Face{ia, ib, ic, normal, glm::dot(normal, A)};
}

void addUniqueEdge(std::vector<std::pair<int, int>>& edges, int a,
                   int b) noexcept {
  for (std::size_t i = 0; i < edges.size(); ++i) {
    if (edges[i].first == b && edges[i].second == a) {
      edges.erase(edges.begin() + static_cast<long>(i));
      return;
    }
  }
  edges.emplace_back(a, b);
}

EpaResult epa3D(const ShapeVariant& shapeA, const Transform& ta,
               const ShapeVariant& shapeB, const Transform& tb,
               const GjkResult& terminalSimplex) noexcept {
  std::vector<SupportPoint> verts(terminalSimplex.simplex.begin(),
                                  terminalSimplex.simplex.begin() + 4);
  Vec3f centroid(0.0f);
  for (const SupportPoint& v : verts) centroid += v.diff;
  centroid *= 0.25f;

  std::vector<Face> faces{
      makeFace(verts, 0, 1, 2, centroid), makeFace(verts, 0, 2, 3, centroid),
      makeFace(verts, 0, 3, 1, centroid), makeFace(verts, 1, 3, 2, centroid)};

  std::size_t closestIdx = 0;
  for (int iter = 0; iter < kMaxIterations; ++iter) {
    closestIdx = 0;
    for (std::size_t i = 1; i < faces.size(); ++i) {
      if (faces[i].distance < faces[closestIdx].distance) closestIdx = i;
    }
    const Face closest = faces[closestIdx];

    SupportPoint support =
        minkowskiSupport(shapeA, ta, shapeB, tb, closest.normal);
    float newDistance = glm::dot(closest.normal, support.diff);

    if (newDistance - closest.distance < kEpaEpsilon) break;

    int newIdx = static_cast<int>(verts.size());
    verts.push_back(support);

    std::vector<std::pair<int, int>> boundaryEdges;
    for (std::size_t i = 0; i < faces.size();) {
      const Face& f = faces[i];
      Vec3f faceVertex = verts[static_cast<std::size_t>(f.a)].diff;
      if (glm::dot(f.normal, support.diff - faceVertex) > 1e-6f) {
        addUniqueEdge(boundaryEdges, f.a, f.b);
        addUniqueEdge(boundaryEdges, f.b, f.c);
        addUniqueEdge(boundaryEdges, f.c, f.a);
        faces.erase(faces.begin() + static_cast<long>(i));
      } else {
        ++i;
      }
    }

    for (const auto& [p, q] : boundaryEdges) {
      faces.push_back(makeFace(verts, p, q, newIdx, centroid));
    }

    assert(!faces.empty() && "EPA polytope expansion produced no faces");
    if (faces.empty()) break;
  }

  const Face& closest = faces[closestIdx];
  const Vec3f& A = verts[static_cast<std::size_t>(closest.a)].diff;
  const Vec3f& B = verts[static_cast<std::size_t>(closest.b)].diff;
  const Vec3f& C = verts[static_cast<std::size_t>(closest.c)].diff;
  Vec3f closestPoint = closest.normal * closest.distance;

  float u, v, w;
  triangleBarycentric(A, B, C, closestPoint, u, v, w);

  const SupportPoint& sa = verts[static_cast<std::size_t>(closest.a)];
  const SupportPoint& sb = verts[static_cast<std::size_t>(closest.b)];
  const SupportPoint& sc = verts[static_cast<std::size_t>(closest.c)];

  EpaResult result;
  result.normal = closest.normal;
  result.penetrationDepth = closest.distance;
  result.contactPointOnA = u * sa.a + v * sb.a + w * sc.a;
  result.contactPointOnB = u * sa.b + v * sb.b + w * sc.b;
  return result;
}

}  // namespace

EpaResult epaPenetration(const ShapeVariant& shapeA, const Transform& ta,
                         const ShapeVariant& shapeB, const Transform& tb,
                         const GjkResult& terminalSimplex) noexcept {
  if (isSphere(shapeA) && isSphere(shapeB)) {
    return sphereVsSphere(std::get<SphereShape>(shapeA), ta,
                          std::get<SphereShape>(shapeB), tb);
  }

  assert(terminalSimplex.overlapping &&
        "epaPenetration requires an overlapping GjkResult");
  assert((terminalSimplex.simplexCount == 3 ||
         terminalSimplex.simplexCount == 4) &&
        "EPA requires GJK's 3- or 4-point terminal simplex");

  if (terminalSimplex.simplexCount == 3) {
    return epa2D(shapeA, ta, shapeB, tb, terminalSimplex);
  }
  return epa3D(shapeA, ta, shapeB, tb, terminalSimplex);
}
