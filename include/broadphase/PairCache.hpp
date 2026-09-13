#ifndef RPENGINE_BROADPHASE_PAIRCACHE_HPP
#define RPENGINE_BROADPHASE_PAIRCACHE_HPP

#include <broadphase/DynamicBVH.hpp>
#include <core/BodyHandle.hpp>
#include <cstdint>
#include <span>
#include <unordered_set>
#include <utility>
#include <vector>

class PairCache {
 public:
  void update(const DynamicBVH& tree, std::span<const int32_t> movedNodeIds);
  std::span<const std::pair<BodyHandle, BodyHandle>> pairs() const noexcept {
    return pairs_;
  }

 private:
  struct Entry {
    int32_t nodeA, nodeB;
    BodyHandle a, b;
  };

  struct NodePairHash {
    size_t operator()(const std::pair<int32_t, int32_t>& p) const noexcept {
      return (static_cast<uint64_t>(p.first) << 32) ^
             static_cast<uint64_t>(p.second);
    }
  };

  static std::pair<BodyHandle, BodyHandle> canonicalize(BodyHandle a,
                                                        BodyHandle b) noexcept;

  std::vector<Entry> entries_;
  std::unordered_set<std::pair<int32_t, int32_t>, NodePairHash> nodePairSet_;
  std::vector<std::pair<BodyHandle, BodyHandle>> pairs_;
};

#endif
