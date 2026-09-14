#include <algorithm>
#include <broadphase/DynamicBVHBroadphase.hpp>
#include <core/BodyStore.hpp>

std::span<const std::pair<BodyHandle, BodyHandle>>
DynamicBVHBroadphase::computePairs(const BodyStore& bodies) {
  movedNodeIds_.clear();

  const std::vector<BodyHandle>& liveHandles = bodies.liveHandles();

  uint32_t maxIndex = 0;
  for (BodyHandle h : liveHandles) { maxIndex = std::max(maxIndex, h.index); }
  if (nodeByBodyIndex_.size() <= maxIndex) {
    nodeByBodyIndex_.resize(maxIndex + 1, kInvalidNode);
    trackedHandles_.resize(maxIndex + 1);
  }

  for (BodyHandle h : liveHandles) {
    int32_t& nodeId = nodeByBodyIndex_[h.index];

    if (nodeId == kInvalidNode || !(trackedHandles_[h.index] == h)) {
      if (nodeId != kInvalidNode) {
        tree_.remove(nodeId);
      } else {
        trackedIndices_.push_back(h.index);  // first time tracking this slot
      }
      nodeId = tree_.insert(h, bodies.aabb(h));
      trackedHandles_[h.index] = h;
      movedNodeIds_.push_back(nodeId);  // newly inserted counts as moved
    } else {
      bool moved =
          tree_.moveProxy(nodeId, bodies.aabb(h), bodies.displacement(h));
      if (moved) { movedNodeIds_.push_back(nodeId); }
    }
  }

  size_t writeIdx = 0;
  for (size_t readIdx = 0; readIdx < trackedIndices_.size(); readIdx++) {
    uint32_t idx = trackedIndices_[readIdx];
    if (bodies.isLive(trackedHandles_[idx])) {
      trackedIndices_[writeIdx++] = idx;
    } else {
      tree_.remove(nodeByBodyIndex_[idx]);
      nodeByBodyIndex_[idx] = kInvalidNode;
    }
  }
  trackedIndices_.resize(writeIdx);

  pairCache_.update(tree_, movedNodeIds_);
  return pairCache_.pairs();
}
