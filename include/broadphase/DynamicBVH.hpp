#ifndef RPENGINE_BROADPHASE_DYNAMICBVH_HPP
#define RPENGINE_BROADPHASE_DYNAMICBVH_HPP

#include <collision/AABB.hpp>
#include <core/BodyHandle.hpp>
#include <cstdint>
#include <math/Types.hpp>
#include <vector>

struct BVHNode {
  AABB fatAABB{Vec3f(0.0f), Vec3f(0.0f)};
  int32_t parent = -1;
  int32_t child1 = -1;
  int32_t child2 = -1;
  int32_t height = 0;  // 0 for leaf, -1 for free-list node, >=1 for internal
                       // nodes
  BodyHandle body{};
  bool isLeaf() const noexcept { return child1 == -1; }
};

class DynamicBVH {
 public:
  int32_t insert(BodyHandle body, const AABB& aabb);
  void remove(int32_t nodeId);

  bool moveProxy(int32_t nodeId, const AABB& realAABB, Vec3f displacement);

  // callback invoked as cb(int32_t nodeId, BodyHandle body) for each leaf whose
  // fatAABB overlaps queryAABB
  template <typename Callback>
  void query(const AABB& queryAABB, Callback&& cb) const {
    if (root_ == -1) return;
    std::vector<int32_t> stack;
    stack.push_back(root_);
    while (!stack.empty()) {
      int32_t nodeId = stack.back();
      stack.pop_back();
      const BVHNode& node = nodes_[nodeId];
      if (!overlaps(queryAABB, node.fatAABB)) continue;
      if (node.isLeaf()) {
        cb(nodeId, node.body);
      } else {
        stack.push_back(node.child1);
        stack.push_back(node.child2);
      }
    }
  }

  const BVHNode& node(int32_t nodeId) const noexcept { return nodes_[nodeId]; }
  int32_t root() const noexcept { return root_; }

  static bool overlaps(const AABB& a, const AABB& b) noexcept;

  // FOR TESTS ONLY!!! only incremented inside insertLeaf/removeLeaf/balance so
  // tests can assert zero tree mutation actually happened, and not just trust
  // moveProxy's returned bool
  uint64_t mutationCount() const noexcept { return mutationCount_; }

 private:
  int32_t allocateNode();
  void freeNode(int32_t nodeId);
  void insertLeaf(int32_t leafId);
  void removeLeaf(int32_t leafId);
  int32_t balance(int32_t nodeId);

  static bool contains(const AABB& outer, const AABB& inner) noexcept;
  static AABB unionOf(const AABB& a, const AABB& b) noexcept;
  static float surfaceArea(const AABB& a) noexcept;
  static AABB fatten(const AABB& realAABB, Vec3f displacement) noexcept;

  static constexpr float kFatMargin = 0.1f;  // world units
  static constexpr float kVelocityPredictionFactor =
      2.0f;  // expands fat AABB along displacement

  std::vector<BVHNode> nodes_;
  int32_t root_ = -1;
  int32_t freeList_ = -1;
  uint64_t mutationCount_ = 0;
};

#endif
