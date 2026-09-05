#ifndef RPENGINE_COLLISION_MANIFOLDCACHE_HPP
#define RPENGINE_COLLISION_MANIFOLDCACHE_HPP

#include <collision/Manifold.hpp>
#include <core/BodyHandle.hpp>
#include <math/Transform.hpp>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

class ManifoldCache {
 public:
  Manifold& getOrCreate(BodyHandle a, BodyHandle b) noexcept;

  void updateManifold(const Manifold& freshManifold, const Transform& transformA,
                      float matchThreshold) noexcept;

  void beginFrame() noexcept;  // mark all entries unvisited
  void endFrame() noexcept;    // erase any entry not visited this frame

  std::span<Manifold> activeManifolds() noexcept;

 private:
  struct PairHash {
    size_t operator()(const std::pair<BodyHandle, BodyHandle>& p) const noexcept;
  };

  static std::pair<BodyHandle, BodyHandle> canonicalize(BodyHandle a,
                                                        BodyHandle b) noexcept;

  std::unordered_map<std::pair<BodyHandle, BodyHandle>, std::size_t, PairHash>
      indexOf_;
  std::vector<Manifold> manifolds_;
  std::vector<bool> visited_;
};

#endif
