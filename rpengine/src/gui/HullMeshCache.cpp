#include <gui/HullMeshCache.hpp>

#include <render/MeshLibrary.hpp>

#include <vector>

namespace gui {

namespace {

render::MeshData triangulateConvexHull(const ConvexHullShape& hull) {
  render::MeshData mesh;
  const auto& vertices = hull.localVertices;
  if (vertices.size() < 3) { return mesh; }

  constexpr float kMinCrossLengthSq = 1e-12f;
  Vec3f cross{0.0f, 0.0f, 0.0f};
  for (size_t i = 1; i + 1 < vertices.size(); ++i) {
    cross = glm::cross(vertices[i] - vertices[0],
                       vertices[i + 1] - vertices[0]);
    if (glm::dot(cross, cross) > kMinCrossLengthSq) { break; }
  }
  if (glm::dot(cross, cross) <= kMinCrossLengthSq) { return mesh; }
  Vec3f normal = glm::normalize(cross);

  for (const Vec3f& vertex : vertices) {
    mesh.vertices.push_back({vertex, normal});
  }
  for (size_t i = 1; i + 1 < vertices.size(); ++i) {
    mesh.indices.push_back(0);
    mesh.indices.push_back(static_cast<uint32_t>(i));
    mesh.indices.push_back(static_cast<uint32_t>(i + 1));
  }
  return mesh;
}

}  // namespace

const render::Mesh& HullMeshCache::getOrBuild(BodyHandle handle,
                                              const ConvexHullShape& hull) {
  return getOrBuild(handle, 0, hull);
}

const render::Mesh& HullMeshCache::getOrBuild(BodyHandle handle,
                                              uint32_t childIndex,
                                              const ConvexHullShape& hull) {
  Key key{handle, childIndex};
  auto it = meshes_.find(key);
  if (it != meshes_.end()) { return it->second; }

  render::MeshData data = triangulateConvexHull(hull);
  auto [inserted, ok] = meshes_.emplace(key, render::Mesh(data));
  (void)ok;
  return inserted->second;
}

void HullMeshCache::pruneDead(const BodyStore& bodies) {
  std::erase_if(meshes_, [&](const auto& entry) {
    return !bodies.isLive(entry.first.handle);
  });
}

}  // namespace gui
