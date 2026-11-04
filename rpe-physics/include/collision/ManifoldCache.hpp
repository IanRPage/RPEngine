#ifndef RPENGINE_COLLISION_MANIFOLDCACHE_HPP
#define RPENGINE_COLLISION_MANIFOLDCACHE_HPP

#include <collision/Manifold.hpp>
#include <cstdint>
#include <core/BodyHandle.hpp>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

class ManifoldCache {
 public:
  Manifold& getOrCreate(BodyHandle a, BodyHandle b) noexcept;
  Manifold& getOrCreate(BodyHandle a, BodyHandle b, uint32_t childA,
                        uint32_t childB) noexcept;

  void updateManifold(const Manifold& freshManifold,
                      float matchThreshold) noexcept;

  void beginFrame() noexcept;  // mark all entries unvisited
  void endFrame() noexcept;    // erase any entry not visited this frame

  std::span<Manifold> activeManifolds() noexcept;

  template <typename Pred>
  void retainIf(Pred&& pred) noexcept {
    for (std::size_t i = 0; i < manifolds_.size(); ++i) {
      if (!visited_[i] && pred(manifolds_[i])) { visited_[i] = true; }
    }
  }

 private:
  struct Key {
    BodyHandle first;
    BodyHandle second;
    uint32_t childFirst = 0;
    uint32_t childSecond = 0;
    bool operator==(const Key& o) const noexcept {
      return first == o.first && second == o.second &&
             childFirst == o.childFirst && childSecond == o.childSecond;
    }
  };

  struct KeyHash {
    size_t operator()(const Key& k) const noexcept;
  };

  static Key canonicalize(BodyHandle a, BodyHandle b, uint32_t childA = 0,
                          uint32_t childB = 0) noexcept;
  static Key keyOf(const Manifold& m) noexcept;

  std::unordered_map<Key, std::size_t, KeyHash> indexOf_;
  std::vector<Manifold> manifolds_;
  std::vector<bool> visited_;
};

#endif
