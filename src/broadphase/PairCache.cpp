#include <broadphase/PairCache.hpp>
#include <utility>

std::pair<BodyHandle, BodyHandle> PairCache::canonicalize(
    BodyHandle a, BodyHandle b) noexcept {
  return (b < a) ? std::make_pair(b, a) : std::make_pair(a, b);
}

void PairCache::update(const DynamicBVH& tree,
                       std::span<const int32_t> movedNodeIds) {
  for (int32_t movedId : movedNodeIds) {
    const BVHNode& movedNode = tree.node(movedId);
    tree.query(movedNode.fatAABB, [&](int32_t otherId, BodyHandle otherBody) {
      if (otherId == movedId) return;

      int32_t nodeA = movedId, nodeB = otherId;
      BodyHandle a = movedNode.body, b = otherBody;
      if (nodeB < nodeA) {
        std::swap(nodeA, nodeB);
        std::swap(a, b);
      }

      auto key = std::make_pair(nodeA, nodeB);
      if (nodePairSet_.count(key) != 0) return;
      nodePairSet_.insert(key);
      entries_.push_back(Entry{nodeA, nodeB, a, b});
    });
  }

  std::vector<Entry> kept;
  kept.reserve(entries_.size());
  for (const Entry& e : entries_) {
    const BVHNode& nodeA = tree.node(e.nodeA);
    const BVHNode& nodeB = tree.node(e.nodeB);

    bool stale = nodeA.height == -1 || nodeA.body != e.a ||
                 nodeB.height == -1 || nodeB.body != e.b;
    if (stale || !DynamicBVH::overlaps(nodeA.fatAABB, nodeB.fatAABB)) {
      nodePairSet_.erase({e.nodeA, e.nodeB});
      continue;
    }
    kept.push_back(e);
  }
  entries_ = std::move(kept);

  pairs_.clear();
  pairs_.reserve(entries_.size());
  for (const Entry& e : entries_) {
    pairs_.push_back(canonicalize(e.a, e.b));
  }
}
