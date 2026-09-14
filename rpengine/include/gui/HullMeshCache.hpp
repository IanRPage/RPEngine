#ifndef RPENGINE_RENDER_HULLMESHCACHE_HPP
#define RPENGINE_RENDER_HULLMESHCACHE_HPP

#include <collision/Shapes.hpp>
#include <core/BodyHandle.hpp>
#include <core/BodyStore.hpp>
#include <render/Mesh.hpp>

#include <unordered_map>

namespace gui {

class HullMeshCache {
 public:
  const render::Mesh& getOrBuild(BodyHandle handle,
                                 const ConvexHullShape& hull);
  void pruneDead(const BodyStore& bodies);
  size_t size() const noexcept { return meshes_.size(); }

 private:
  std::unordered_map<BodyHandle, render::Mesh, BodyHandleHash> meshes_;
};

}  // namespace gui

#endif
