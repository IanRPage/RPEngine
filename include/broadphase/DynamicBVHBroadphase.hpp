#ifndef RPENGINE_BROADPHASE_DYNAMICBVHBROADPHASE_HPP
#define RPENGINE_BROADPHASE_DYNAMICBVHBROADPHASE_HPP

#include <broadphase/DynamicBVH.hpp>
#include <broadphase/IBroadphase.hpp>
#include <broadphase/PairCache.hpp>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

class DynamicBVHBroadphase final : public IBroadphase {
 public:
  std::span<const std::pair<BodyHandle, BodyHandle>> computePairs(
      const BodyStore& bodies) override;

  const DynamicBVH& tree() const noexcept { return tree_; }

 private:
  DynamicBVH tree_;
  PairCache pairCache_;

  static constexpr int32_t kInvalidNode = -1;
  std::vector<int32_t> nodeByBodyIndex_;
  std::vector<BodyHandle> trackedHandles_;

  std::vector<int32_t> movedNodeIds_;
};

#endif
