#include <algorithm>
#include <broadphase/DynamicBVHBroadphase.hpp>
#include <core/BodyStore.hpp>

std::span<const std::pair<BodyHandle, BodyHandle>>
DynamicBVHBroadphase::computePairs(const BodyStore& bodies) {
  movedNodeIds_.clear();

  const std::vector<BodyHandle>& liveHandles = bodies.liveHandles();

  uint32_t maxIndex = 0;
  for (BodyHandle h : liveHandles) maxIndex = std::max(maxIndex, h.index);
  if (nodeByBodyIndex_.size() <= maxIndex) {
    nodeByBodyIndex_.resize(maxIndex + 1, kInvalidNode);
    trackedHandles_.resize(maxIndex + 1);
  }

  std::vector<bool> seen(nodeByBodyIndex_.size(), false);

  for (BodyHandle h : liveHandles) {
    seen[h.index] = true;
    int32_t& nodeId = nodeByBodyIndex_[h.index];

    if (nodeId == kInvalidNode || !(trackedHandles_[h.index] == h)) {
      if (nodeId != kInvalidNode) tree_.remove(nodeId);
      nodeId = tree_.insert(h, bodies.aabb(h));
      trackedHandles_[h.index] = h;
      movedNodeIds_.push_back(nodeId);  // newly inserted counts as moved
    } else {
      bool moved =
          tree_.moveProxy(nodeId, bodies.aabb(h), bodies.displacement(h));
      if (moved) movedNodeIds_.push_back(nodeId);
    }
  }

  for (uint32_t i = 0; i < nodeByBodyIndex_.size(); i++) {
    if (!seen[i] && nodeByBodyIndex_[i] != kInvalidNode) {
      tree_.remove(nodeByBodyIndex_[i]);
      nodeByBodyIndex_[i] = kInvalidNode;
    }
  }

  pairCache_.update(tree_, movedNodeIds_);
  return pairCache_.pairs();
}
