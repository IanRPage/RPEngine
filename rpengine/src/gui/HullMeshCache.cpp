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
  auto it = meshes_.find(handle);
  if (it != meshes_.end()) { return it->second; }

  render::MeshData data = triangulateConvexHull(hull);
  auto [inserted, ok] = meshes_.emplace(handle, render::Mesh(data));
  (void)ok;
  return inserted->second;
}

void HullMeshCache::pruneDead(const BodyStore& bodies) {
  std::vector<BodyHandle> dead;
  for (const auto& [handle, mesh] : meshes_) {
    if (!bodies.isLive(handle)) { dead.push_back(handle); }
  }
  for (BodyHandle handle : dead) { meshes_.erase(handle); }
}

}  // namespace gui
