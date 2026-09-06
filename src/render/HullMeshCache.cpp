#include <render/HullMeshCache.hpp>

#include <render/MeshLibrary.hpp>

#include <vector>

namespace render {

const Mesh& HullMeshCache::getOrBuild(BodyHandle handle,
                                      const ConvexHullShape& hull) {
  auto it = meshes_.find(handle);
  if (it != meshes_.end()) { return it->second; }

  MeshData data = triangulateConvexHull(hull);
  auto [inserted, ok] = meshes_.emplace(handle, Mesh(data));
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

}  // namespace render
