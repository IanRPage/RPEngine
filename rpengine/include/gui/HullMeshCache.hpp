#ifndef RPENGINE_RENDER_HULLMESHCACHE_HPP
#define RPENGINE_RENDER_HULLMESHCACHE_HPP

#include <collision/Shapes.hpp>
#include <core/BodyHandle.hpp>
#include <core/BodyStore.hpp>
#include <render/Mesh.hpp>

#include <cstdint>
#include <unordered_map>

namespace gui {

class HullMeshCache {
 public:
  const render::Mesh& getOrBuild(BodyHandle handle,
                                 const ConvexHullShape& hull);
  const render::Mesh& getOrBuild(BodyHandle handle, uint32_t childIndex,
                                 const ConvexHullShape& hull);
  void pruneDead(const BodyStore& bodies);
  size_t size() const noexcept { return meshes_.size(); }

 private:
  struct Key {
    BodyHandle handle;
    uint32_t childIndex = 0;
    bool operator==(const Key&) const noexcept = default;
  };
  struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
      size_t h = BodyHandleHash{}(k.handle);
      return h ^ (std::hash<uint32_t>{}(k.childIndex) + 0x9e3779b9 +
                  (h << 6) + (h >> 2));
    }
  };

  std::unordered_map<Key, render::Mesh, KeyHash> meshes_;
};

}  // namespace gui

#endif
