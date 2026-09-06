#include <collision/Manifold.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <math/Constants.hpp>
#include <vector>

namespace {

constexpr float kPenetrationTolerance = 1e-3f;

bool isSphere(const ShapeVariant& shape) noexcept {
  return std::holds_alternative<SphereShape>(shape);
}

bool isBox(const ShapeVariant& shape) noexcept {
  return std::holds_alternative<BoxShape>(shape);
}

ManifoldPoint makeManifoldPoint(const Transform& ta, const Transform& tb,
                                Vec3f worldOnA, Vec3f worldOnB,
                                float penetration) noexcept {
  ManifoldPoint p;
  p.localAnchorA = transformPoint(inverse(ta), worldOnA);
  p.localAnchorB = transformPoint(inverse(tb), worldOnB);
  p.penetration = penetration;
  return p;
}

Manifold singlePointManifold(const Transform& ta, const Transform& tb,
                             BodyHandle bodyA, BodyHandle bodyB,
                             const EpaResult& epa) noexcept {
  Manifold m;
  m.bodyA = bodyA;
  m.bodyB = bodyB;
  m.normal = epa.normal;
  m.points[0] = makeManifoldPoint(ta, tb, epa.contactPointOnA,
                                  epa.contactPointOnB, epa.penetrationDepth);
  m.pointCount = 1;
  return m;
}

// ---- 2D clipping: shapes reduced to CCW world-space polygons in z=0 ----

struct Edge2D {
  Vec3f a, b;
  Vec3f normal;  // outward
};

float signedArea2D(const std::vector<Vec3f>& poly) noexcept {
  float area = 0.0f;
  std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t j = (i + 1) % n;
    area += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
  }
  return 0.5f * area;
}

std::vector<Vec3f> shapeWorldPolygon2D(const ShapeVariant& shape,
                                       const Transform& t) noexcept {
  std::vector<Vec3f> verts;
  if (const auto* box = std::get_if<BoxShape>(&shape)) {
    float hx = box->halfExtents.x, hy = box->halfExtents.y;
    verts = {Vec3f(-hx, -hy, 0.0f), Vec3f(hx, -hy, 0.0f), Vec3f(hx, hy, 0.0f),
            Vec3f(-hx, hy, 0.0f)};
  } else if (const auto* hull = std::get_if<ConvexHullShape>(&shape)) {
    verts = hull->localVertices;
  }
  for (Vec3f& v : verts) v = transformPoint(t, v);
  if (verts.size() >= 3 && signedArea2D(verts) < 0.0f) {
    std::reverse(verts.begin(), verts.end());
  }
  return verts;
}

Edge2D bestAlignedEdge2D(const std::vector<Vec3f>& poly,
                         Vec3f targetDir) noexcept {
  Edge2D best{Vec3f(0.0f), Vec3f(0.0f), Vec3f(0.0f)};
  float bestDot = -std::numeric_limits<float>::max();
  std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t j = (i + 1) % n;
    Vec3f e = poly[j] - poly[i];
    float len = glm::length(e);
    if (len < VECTOR_LENGTH_EPSILON) continue;
    Vec3f normal(e.y / len, -e.x / len, 0.0f);
    float d = glm::dot(normal, targetDir);
    if (d > bestDot) {
      bestDot = d;
      best = Edge2D{poly[i], poly[j], normal};
    }
  }
  return best;
}

std::vector<Vec3f> clipSegmentAgainstPlane(Vec3f p0, Vec3f p1,
                                           Vec3f planePoint,
                                           Vec3f planeNormal) noexcept {
  float d0 = glm::dot(planeNormal, p0 - planePoint);
  float d1 = glm::dot(planeNormal, p1 - planePoint);
  bool in0 = d0 >= 0.0f;
  bool in1 = d1 >= 0.0f;
  if (in0 && in1) return {p0, p1};
  if (!in0 && !in1) return {};
  float t = d0 / (d0 - d1);
  Vec3f intersection = p0 + t * (p1 - p0);
  return in0 ? std::vector<Vec3f>{p0, intersection}
             : std::vector<Vec3f>{intersection, p1};
}

std::vector<Vec3f> clipPolygonAgainstPlane(const std::vector<Vec3f>& poly,
                                           Vec3f planePoint,
                                           Vec3f planeNormal) noexcept {
  std::vector<Vec3f> out;
  std::size_t n = poly.size();
  if (n == 0) return out;
  for (std::size_t i = 0; i < n; ++i) {
    Vec3f cur = poly[i];
    Vec3f prev = poly[(i + n - 1) % n];
    float curDist = glm::dot(planeNormal, cur - planePoint);
    float prevDist = glm::dot(planeNormal, prev - planePoint);
    bool curIn = curDist >= 0.0f;
    bool prevIn = prevDist >= 0.0f;
    if (curIn) {
      if (!prevIn) {
        float t = prevDist / (prevDist - curDist);
        out.push_back(prev + t * (cur - prev));
      }
      out.push_back(cur);
    } else if (prevIn) {
      float t = prevDist / (prevDist - curDist);
      out.push_back(prev + t * (cur - prev));
    }
  }
  return out;
}

Manifold clip2D(const ShapeVariant& shapeA, const Transform& ta,
               const ShapeVariant& shapeB, const Transform& tb,
               BodyHandle bodyA, BodyHandle bodyB,
               const EpaResult& epa) noexcept {
  std::vector<Vec3f> polyA = shapeWorldPolygon2D(shapeA, ta);
  std::vector<Vec3f> polyB = shapeWorldPolygon2D(shapeB, tb);

  if (polyA.size() < 2 || polyB.size() < 2) {
    return singlePointManifold(ta, tb, bodyA, bodyB, epa);
  }

  Edge2D edgeA = bestAlignedEdge2D(polyA, epa.normal);
  Edge2D edgeB = bestAlignedEdge2D(polyB, -epa.normal);

  bool aIsReference = glm::dot(edgeA.normal, epa.normal) >=
                      glm::dot(edgeB.normal, -epa.normal);
  const Edge2D& refEdge = aIsReference ? edgeA : edgeB;
  const Edge2D& incEdge = aIsReference ? edgeB : edgeA;

  Vec3f edgeDir = glm::normalize(refEdge.b - refEdge.a);
  std::vector<Vec3f> clipped =
      clipSegmentAgainstPlane(incEdge.a, incEdge.b, refEdge.a, edgeDir);
  if (!clipped.empty()) {
    clipped =
        clipSegmentAgainstPlane(clipped[0], clipped[1], refEdge.b, -edgeDir);
  }

  Manifold m;
  m.bodyA = bodyA;
  m.bodyB = bodyB;
  m.normal = epa.normal;

  for (const Vec3f& p : clipped) {
    if (m.pointCount >= m.points.size()) break;
    float penetration = glm::dot(refEdge.normal, refEdge.a - p);
    if (penetration < -kPenetrationTolerance) continue;
    Vec3f projected = p + refEdge.normal * penetration;
    Vec3f worldOnRef = projected;
    Vec3f worldOnInc = p;
    Vec3f worldOnA = aIsReference ? worldOnRef : worldOnInc;
    Vec3f worldOnB = aIsReference ? worldOnInc : worldOnRef;
    m.points[m.pointCount++] =
        makeManifoldPoint(ta, tb, worldOnA, worldOnB, penetration);
  }

  if (m.pointCount == 0) return singlePointManifold(ta, tb, bodyA, bodyB, epa);
  return m;
}

// ---- 3D box-vs-box clipping: each box reduced to 6 quad faces ----

struct Face3D {
  std::array<Vec3f, 4> verts;  // CCW viewed from outside along normal
  Vec3f normal;                // outward
};

std::array<Face3D, 6> boxWorldFaces(const BoxShape& box,
                                    const Transform& t) noexcept {
  Vec3f h = box.halfExtents;
  auto worldPoint = [&](float sx, float sy, float sz) {
    return transformPoint(t, Vec3f(sx * h.x, sy * h.y, sz * h.z));
  };
  auto worldDir = [&](Vec3f localN) {
    return transformDirection(t, localN);
  };

  return std::array<Face3D, 6>{
      Face3D{{worldPoint(1, -1, -1), worldPoint(1, 1, -1),
             worldPoint(1, 1, 1), worldPoint(1, -1, 1)},
            worldDir(Vec3f(1, 0, 0))},
      Face3D{{worldPoint(-1, -1, 1), worldPoint(-1, 1, 1),
             worldPoint(-1, 1, -1), worldPoint(-1, -1, -1)},
            worldDir(Vec3f(-1, 0, 0))},
      Face3D{{worldPoint(-1, 1, -1), worldPoint(-1, 1, 1),
             worldPoint(1, 1, 1), worldPoint(1, 1, -1)},
            worldDir(Vec3f(0, 1, 0))},
      Face3D{{worldPoint(-1, -1, 1), worldPoint(-1, -1, -1),
             worldPoint(1, -1, -1), worldPoint(1, -1, 1)},
            worldDir(Vec3f(0, -1, 0))},
      Face3D{{worldPoint(-1, -1, 1), worldPoint(1, -1, 1),
             worldPoint(1, 1, 1), worldPoint(-1, 1, 1)},
            worldDir(Vec3f(0, 0, 1))},
      Face3D{{worldPoint(1, -1, -1), worldPoint(-1, -1, -1),
             worldPoint(-1, 1, -1), worldPoint(1, 1, -1)},
            worldDir(Vec3f(0, 0, -1))},
  };
}

const Face3D& bestAlignedFace3D(const std::array<Face3D, 6>& faces,
                                Vec3f targetDir) noexcept {
  std::size_t bestIdx = 0;
  float bestDot = -std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < faces.size(); ++i) {
    float d = glm::dot(faces[i].normal, targetDir);
    if (d > bestDot) {
      bestDot = d;
      bestIdx = i;
    }
  }
  return faces[bestIdx];
}

Manifold clip3DBox(const BoxShape& boxA, const Transform& ta,
                   const BoxShape& boxB, const Transform& tb, BodyHandle bodyA,
                   BodyHandle bodyB, const EpaResult& epa) noexcept {
  std::array<Face3D, 6> facesA = boxWorldFaces(boxA, ta);
  std::array<Face3D, 6> facesB = boxWorldFaces(boxB, tb);

  const Face3D& faceA = bestAlignedFace3D(facesA, epa.normal);
  const Face3D& faceB = bestAlignedFace3D(facesB, -epa.normal);

  bool aIsReference =
      glm::dot(faceA.normal, epa.normal) >= glm::dot(faceB.normal, -epa.normal);
  const Face3D& refFace = aIsReference ? faceA : faceB;
  const Face3D& incFace = aIsReference ? faceB : faceA;

  std::vector<Vec3f> poly(incFace.verts.begin(), incFace.verts.end());
  for (std::size_t i = 0; i < refFace.verts.size(); ++i) {
    std::size_t j = (i + 1) % refFace.verts.size();
    Vec3f edgeDir = refFace.verts[j] - refFace.verts[i];
    Vec3f inward = glm::cross(refFace.normal, edgeDir);
    poly = clipPolygonAgainstPlane(poly, refFace.verts[i], inward);
    if (poly.empty()) break;
  }

  struct Candidate {
    Vec3f point;
    float penetration;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(poly.size());
  for (const Vec3f& p : poly) {
    float penetration = glm::dot(refFace.normal, refFace.verts[0] - p);
    if (penetration < -kPenetrationTolerance) continue;
    candidates.push_back({p, penetration});
  }
  if (candidates.size() > 4) {
    std::sort(candidates.begin(), candidates.end(),
             [](const Candidate& lhs, const Candidate& rhs) {
               return lhs.penetration > rhs.penetration;
             });
    candidates.resize(4);
  }

  Manifold m;
  m.bodyA = bodyA;
  m.bodyB = bodyB;
  m.normal = epa.normal;

  for (const Candidate& c : candidates) {
    Vec3f projected = c.point + refFace.normal * c.penetration;
    Vec3f worldOnA = aIsReference ? projected : c.point;
    Vec3f worldOnB = aIsReference ? c.point : projected;
    m.points[m.pointCount++] =
        makeManifoldPoint(ta, tb, worldOnA, worldOnB, c.penetration);
  }

  if (m.pointCount == 0) return singlePointManifold(ta, tb, bodyA, bodyB, epa);
  return m;
}

}  // namespace

Manifold buildManifold(const ShapeVariant& shapeA, const Transform& ta,
                       const ShapeVariant& shapeB, const Transform& tb,
                       BodyHandle bodyA, BodyHandle bodyB,
                       const GjkResult& terminalSimplex,
                       const EpaResult& epa) noexcept {
  if (isSphere(shapeA) || isSphere(shapeB)) {
    return singlePointManifold(ta, tb, bodyA, bodyB, epa);
  }

  bool is2D = isFlatPair(shapeA, ta, shapeB, tb) || terminalSimplex.simplexCount == 3;
  if (is2D) {
    return clip2D(shapeA, ta, shapeB, tb, bodyA, bodyB, epa);
  }

  if (isBox(shapeA) && isBox(shapeB)) {
    return clip3DBox(std::get<BoxShape>(shapeA), ta, std::get<BoxShape>(shapeB),
                     tb, bodyA, bodyB, epa);
  }

  return singlePointManifold(ta, tb, bodyA, bodyB, epa);
}
