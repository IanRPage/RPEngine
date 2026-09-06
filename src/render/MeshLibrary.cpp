#include <render/MeshLibrary.hpp>

#include <array>
#include <cmath>
#include <map>
#include <utility>

namespace render {

namespace {

constexpr float kPi = 3.14159265358979323846f;

void addFace(MeshData& mesh, Vec3f a, Vec3f b, Vec3f c, Vec3f d, Vec3f normal) {
  uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
  mesh.vertices.push_back({a, normal});
  mesh.vertices.push_back({b, normal});
  mesh.vertices.push_back({c, normal});
  mesh.vertices.push_back({d, normal});
  mesh.indices.push_back(base + 0);
  mesh.indices.push_back(base + 1);
  mesh.indices.push_back(base + 2);
  mesh.indices.push_back(base + 2);
  mesh.indices.push_back(base + 3);
  mesh.indices.push_back(base + 0);
}

}  // namespace

MeshData buildUnitQuad() noexcept {
  MeshData mesh;
  addFace(mesh, {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
          {-1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
  return mesh;
}

MeshData buildUnitCircle(int segments) noexcept {
  MeshData mesh;
  mesh.vertices.push_back({{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}});
  for (int i = 0; i < segments; ++i) {
    float theta =
        2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
    mesh.vertices.push_back(
        {{std::cos(theta), std::sin(theta), 0.0f}, {0.0f, 0.0f, 1.0f}});
  }
  for (int i = 0; i < segments; ++i) {
    uint32_t curr = static_cast<uint32_t>(1 + i);
    uint32_t next = static_cast<uint32_t>(1 + (i + 1) % segments);
    mesh.indices.push_back(0);
    mesh.indices.push_back(curr);
    mesh.indices.push_back(next);
  }
  return mesh;
}

MeshData buildUnitIcosphere(int subdivisions) noexcept {
  const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;

  std::vector<Vec3f> positions = {
      {-1, t, 0},  {1, t, 0},  {-1, -t, 0}, {1, -t, 0}, {0, -1, t},  {0, 1, t},
      {0, -1, -t}, {0, 1, -t}, {t, 0, -1},  {t, 0, 1},  {-t, 0, -1}, {-t, 0, 1},
  };
  for (Vec3f& p : positions) { p = glm::normalize(p); }

  std::vector<std::array<uint32_t, 3>> faces = {
      {0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
      {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
      {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
      {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1},
  };

  std::map<std::pair<uint32_t, uint32_t>, uint32_t> midpointCache;
  auto midpoint = [&](uint32_t a, uint32_t b) -> uint32_t {
    auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    auto it = midpointCache.find(key);
    if (it != midpointCache.end()) { return it->second; }
    Vec3f mid = glm::normalize(positions[a] + positions[b]);
    uint32_t index = static_cast<uint32_t>(positions.size());
    positions.push_back(mid);
    midpointCache.emplace(key, index);
    return index;
  };

  for (int s = 0; s < subdivisions; ++s) {
    std::vector<std::array<uint32_t, 3>> nextFaces;
    nextFaces.reserve(faces.size() * 4);
    for (const auto& f : faces) {
      uint32_t ab = midpoint(f[0], f[1]);
      uint32_t bc = midpoint(f[1], f[2]);
      uint32_t ca = midpoint(f[2], f[0]);
      nextFaces.push_back({f[0], ab, ca});
      nextFaces.push_back({f[1], bc, ab});
      nextFaces.push_back({f[2], ca, bc});
      nextFaces.push_back({ab, bc, ca});
    }
    faces = std::move(nextFaces);
  }

  MeshData mesh;
  mesh.vertices.reserve(positions.size());
  for (const Vec3f& p : positions) {
    mesh.vertices.push_back(
        {p, p});  // unit sphere at origin, normal == position
  }
  mesh.indices.reserve(faces.size() * 3);
  for (const auto& f : faces) {
    mesh.indices.push_back(f[0]);
    mesh.indices.push_back(f[1]);
    mesh.indices.push_back(f[2]);
  }
  return mesh;
}

MeshData buildUnitBox() noexcept {
  MeshData mesh;
  // +X, -X, +Y, -Y, +Z, -Z faces, each wound CCW when viewed from outside
  addFace(mesh, {1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}, {1, 0, 0});
  addFace(mesh, {-1, -1, 1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, -1}, {-1, 0, 0});
  addFace(mesh, {-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}, {1, 1, -1}, {0, 1, 0});
  addFace(mesh, {-1, -1, 1}, {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {0, -1, 0});
  addFace(mesh, {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}, {0, 0, 1});
  addFace(mesh, {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {0, 0, -1});
  return mesh;
}

MeshData buildUnitCapsule(int radialSegments, int capRings) noexcept {
  MeshData mesh;

  auto ringVertex = [&](float theta, float y, float radius, Vec3f normal) {
    mesh.vertices.push_back(
        {{radius * std::cos(theta), y, radius * std::sin(theta)}, normal});
  };

  uint32_t cylBase = static_cast<uint32_t>(mesh.vertices.size());
  for (int i = 0; i <= radialSegments; ++i) {
    float theta =
        2.0f * kPi * static_cast<float>(i) / static_cast<float>(radialSegments);
    Vec3f n{std::cos(theta), 0.0f, std::sin(theta)};
    ringVertex(theta, -1.0f, 1.0f, n);
    ringVertex(theta, 1.0f, 1.0f, n);
  }
  for (int i = 0; i < radialSegments; ++i) {
    uint32_t a = cylBase + static_cast<uint32_t>(i) * 2;
    uint32_t b = a + 1;
    uint32_t c = a + 2;
    uint32_t d = a + 3;
    mesh.indices.insert(mesh.indices.end(), {a, c, d, d, b, a});
  }

  auto buildCap = [&](float sign) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int r = 0; r < capRings; ++r) {
      float phi =
          (kPi / 2.0f) * static_cast<float>(r) / static_cast<float>(capRings);
      float ringRadius = std::cos(phi);
      float ringY = sign * std::sin(phi);
      for (int i = 0; i <= radialSegments; ++i) {
        float theta = 2.0f * kPi * static_cast<float>(i) /
                      static_cast<float>(radialSegments);
        Vec3f localPoint{ringRadius * std::cos(theta), ringY,
                         ringRadius * std::sin(theta)};
        Vec3f normal = glm::normalize(localPoint);
        Vec3f position = localPoint + Vec3f(0.0f, sign * 1.0f, 0.0f);
        mesh.vertices.push_back({position, normal});
      }
    }
    uint32_t poleIndex = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(
        {{0.0f, sign * 2.0f, 0.0f}, {0.0f, sign * 1.0f, 0.0f}});

    for (int r = 0; r < capRings - 1; ++r) {
      uint32_t ringA = base + static_cast<uint32_t>(r) * (radialSegments + 1);
      uint32_t ringB =
          base + static_cast<uint32_t>(r + 1) * (radialSegments + 1);
      for (int i = 0; i < radialSegments; ++i) {
        uint32_t a = ringA + static_cast<uint32_t>(i);
        uint32_t b = ringA + static_cast<uint32_t>(i) + 1;
        uint32_t c = ringB + static_cast<uint32_t>(i);
        uint32_t d = ringB + static_cast<uint32_t>(i) + 1;
        if (sign > 0.0f) {
          mesh.indices.insert(mesh.indices.end(), {a, c, d, d, b, a});
        } else {
          mesh.indices.insert(mesh.indices.end(), {a, b, d, d, c, a});
        }
      }
    }
    uint32_t lastRing =
        base + static_cast<uint32_t>(capRings - 1) * (radialSegments + 1);
    for (int i = 0; i < radialSegments; ++i) {
      uint32_t a = lastRing + static_cast<uint32_t>(i);
      uint32_t b = lastRing + static_cast<uint32_t>(i) + 1;
      if (sign > 0.0f) {
        mesh.indices.insert(mesh.indices.end(), {a, b, poleIndex});
      } else {
        mesh.indices.insert(mesh.indices.end(), {b, a, poleIndex});
      }
    }
  };

  buildCap(1.0f);
  buildCap(-1.0f);

  return mesh;
}

MeshData triangulateConvexHull(const ConvexHullShape& hull) noexcept {
  MeshData mesh;
  const auto& verts = hull.localVertices;
  if (verts.size() < 3) { return mesh; }

  Vec3f normal =
      glm::normalize(glm::cross(verts[1] - verts[0], verts[2] - verts[0]));
  for (const Vec3f& v : verts) { mesh.vertices.push_back({v, normal}); }
  for (size_t i = 1; i + 1 < verts.size(); ++i) {
    mesh.indices.push_back(0);
    mesh.indices.push_back(static_cast<uint32_t>(i));
    mesh.indices.push_back(static_cast<uint32_t>(i + 1));
  }
  return mesh;
}

}  // namespace render
