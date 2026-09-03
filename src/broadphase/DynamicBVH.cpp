#include <algorithm>
#include <broadphase/DynamicBVH.hpp>

bool DynamicBVH::overlaps(const AABB& a, const AABB& b) noexcept {
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y &&
         a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

bool DynamicBVH::contains(const AABB& outer, const AABB& inner) noexcept {
  return outer.min.x <= inner.min.x && outer.max.x >= inner.max.x &&
         outer.min.y <= inner.min.y && outer.max.y >= inner.max.y &&
         outer.min.z <= inner.min.z && outer.max.z >= inner.max.z;
}

AABB DynamicBVH::unionOf(const AABB& a, const AABB& b) noexcept {
  return AABB(glm::min(a.min, b.min), glm::max(a.max, b.max));
}

float DynamicBVH::surfaceArea(const AABB& a) noexcept {
  Vec3f d = a.max - a.min;
  return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

AABB DynamicBVH::fatten(const AABB& realAABB, Vec3f displacement) noexcept {
  Vec3f margin(kFatMargin);
  Vec3f mn = realAABB.min - margin;
  Vec3f mx = realAABB.max + margin;

  Vec3f predicted = displacement * kVelocityPredictionFactor;
  mn = glm::min(mn, mn + predicted);
  mx = glm::max(mx, mx + predicted);

  return AABB(mn, mx);
}

int32_t DynamicBVH::allocateNode() {
  int32_t nodeId;
  if (freeList_ != -1) {
    nodeId = freeList_;
    freeList_ = nodes_[nodeId].parent;  // parent doubles as "next free" link
  } else {
    nodeId = static_cast<int32_t>(nodes_.size());
    nodes_.emplace_back();
  }

  nodes_[nodeId] = BVHNode{};
  return nodeId;
}

void DynamicBVH::freeNode(int32_t nodeId) {
  nodes_[nodeId].height = -1;
  nodes_[nodeId].parent = freeList_;
  freeList_ = nodeId;
}

int32_t DynamicBVH::insert(BodyHandle body, const AABB& aabb) {
  int32_t leafId = allocateNode();
  nodes_[leafId].fatAABB = fatten(aabb, Vec3f(0.0f));
  nodes_[leafId].body = body;
  nodes_[leafId].height = 0;

  insertLeaf(leafId);
  return leafId;
}

void DynamicBVH::remove(int32_t nodeId) {
  removeLeaf(nodeId);
  freeNode(nodeId);
}

bool DynamicBVH::moveProxy(int32_t nodeId, const AABB& realAABB,
                           Vec3f displacement) {
  if (contains(nodes_[nodeId].fatAABB, realAABB)) return false;

  removeLeaf(nodeId);
  nodes_[nodeId].fatAABB = fatten(realAABB, displacement);
  insertLeaf(nodeId);
  return true;
}

void DynamicBVH::insertLeaf(int32_t leafId) {
  mutationCount_++;

  if (root_ == -1) {
    root_ = leafId;
    nodes_[leafId].parent = -1;
    return;
  }

  const AABB leafAABB = nodes_[leafId].fatAABB;

  // descend at each internal node picking whichever child minimizes total cost:
  // cost of making the leaf a sibling of that child's subtree plus the
  // inherited cost of descending past this node (discourages growing
  // already-large ancestors)
  int32_t index = root_;
  while (!nodes_[index].isLeaf()) {
    int32_t child1 = nodes_[index].child1;
    int32_t child2 = nodes_[index].child2;

    float area = surfaceArea(nodes_[index].fatAABB);
    AABB combinedAABB = unionOf(nodes_[index].fatAABB, leafAABB);
    float combinedArea = surfaceArea(combinedAABB);

    float cost = 2.0f * combinedArea;
    float inheritanceCost = 2.0f * (combinedArea - area);

    auto descendCost = [&](int32_t child) {
      AABB combined = unionOf(leafAABB, nodes_[child].fatAABB);
      if (nodes_[child].isLeaf()) {
        return surfaceArea(combined) + inheritanceCost;
      }
      float oldArea = surfaceArea(nodes_[child].fatAABB);
      float newArea = surfaceArea(combined);
      return (newArea - oldArea) + inheritanceCost;
    };

    float cost1 = descendCost(child1);
    float cost2 = descendCost(child2);

    if (cost < cost1 && cost < cost2) break;

    index = (cost1 < cost2) ? child1 : child2;
  }

  int32_t sibling = index;
  int32_t oldParent = nodes_[sibling].parent;
  int32_t newParent = allocateNode();

  nodes_[newParent].parent = oldParent;
  nodes_[newParent].fatAABB = unionOf(leafAABB, nodes_[sibling].fatAABB);
  nodes_[newParent].height = nodes_[sibling].height + 1;
  nodes_[newParent].child1 = sibling;
  nodes_[newParent].child2 = leafId;

  nodes_[sibling].parent = newParent;
  nodes_[leafId].parent = newParent;

  if (oldParent != -1) {
    if (nodes_[oldParent].child1 == sibling) {
      nodes_[oldParent].child1 = newParent;
    } else {
      nodes_[oldParent].child2 = newParent;
    }
  } else {
    root_ = newParent;
  }

  // walk back up fixing each ancestor's fatAABB/height and balancing along the
  // way
  index = nodes_[leafId].parent;
  while (index != -1) {
    index = balance(index);

    int32_t child1 = nodes_[index].child1;
    int32_t child2 = nodes_[index].child2;

    nodes_[index].height =
        1 + std::max(nodes_[child1].height, nodes_[child2].height);
    nodes_[index].fatAABB =
        unionOf(nodes_[child1].fatAABB, nodes_[child2].fatAABB);

    index = nodes_[index].parent;
  }
}

void DynamicBVH::removeLeaf(int32_t leafId) {
  mutationCount_++;

  if (leafId == root_) {
    root_ = -1;
    return;
  }

  int32_t parent = nodes_[leafId].parent;
  int32_t grandParent = nodes_[parent].parent;
  int32_t sibling = (nodes_[parent].child1 == leafId) ? nodes_[parent].child2
                                                      : nodes_[parent].child1;

  if (grandParent != -1) {
    if (nodes_[grandParent].child1 == parent) {
      nodes_[grandParent].child1 = sibling;
    } else {
      nodes_[grandParent].child2 = sibling;
    }
    nodes_[sibling].parent = grandParent;
    freeNode(parent);

    int32_t index = grandParent;
    while (index != -1) {
      index = balance(index);

      int32_t child1 = nodes_[index].child1;
      int32_t child2 = nodes_[index].child2;

      nodes_[index].fatAABB =
          unionOf(nodes_[child1].fatAABB, nodes_[child2].fatAABB);
      nodes_[index].height =
          1 + std::max(nodes_[child1].height, nodes_[child2].height);

      index = nodes_[index].parent;
    }
  } else {
    root_ = sibling;
    nodes_[sibling].parent = -1;
    freeNode(parent);
  }
}

int32_t DynamicBVH::balance(int32_t iA) {
  BVHNode& A = nodes_[iA];
  if (A.isLeaf() || A.height < 2) return iA;

  int32_t iB = A.child1;
  int32_t iC = A.child2;
  BVHNode& B = nodes_[iB];
  BVHNode& C = nodes_[iC];

  int32_t balanceFactor = C.height - B.height;

  // rotate C up (left-heavy on right side)
  if (balanceFactor > 1) {
    mutationCount_++;
    int32_t iF = C.child1;
    int32_t iG = C.child2;
    BVHNode& F = nodes_[iF];
    BVHNode& G = nodes_[iG];

    C.child1 = iA;
    C.parent = A.parent;
    A.parent = iC;

    if (C.parent != -1) {
      if (nodes_[C.parent].child1 == iA) {
        nodes_[C.parent].child1 = iC;
      } else {
        nodes_[C.parent].child2 = iC;
      }
    } else {
      root_ = iC;
    }

    if (F.height > G.height) {
      C.child2 = iF;
      A.child2 = iG;
      G.parent = iA;
      A.fatAABB = unionOf(B.fatAABB, G.fatAABB);
      C.fatAABB = unionOf(A.fatAABB, F.fatAABB);
      A.height = 1 + std::max(B.height, G.height);
      C.height = 1 + std::max(A.height, F.height);
    } else {
      C.child2 = iG;
      A.child2 = iF;
      F.parent = iA;
      A.fatAABB = unionOf(B.fatAABB, F.fatAABB);
      C.fatAABB = unionOf(A.fatAABB, G.fatAABB);
      A.height = 1 + std::max(B.height, F.height);
      C.height = 1 + std::max(A.height, G.height);
    }

    return iC;
  }

  // rotate B up (left-heavy on left side)
  if (balanceFactor < -1) {
    mutationCount_++;
    int32_t iD = B.child1;
    int32_t iE = B.child2;
    BVHNode& D = nodes_[iD];
    BVHNode& E = nodes_[iE];

    B.child1 = iA;
    B.parent = A.parent;
    A.parent = iB;

    if (B.parent != -1) {
      if (nodes_[B.parent].child1 == iA) {
        nodes_[B.parent].child1 = iB;
      } else {
        nodes_[B.parent].child2 = iB;
      }
    } else {
      root_ = iB;
    }

    if (D.height > E.height) {
      B.child2 = iD;
      A.child1 = iE;
      E.parent = iA;
      A.fatAABB = unionOf(C.fatAABB, E.fatAABB);
      B.fatAABB = unionOf(A.fatAABB, D.fatAABB);
      A.height = 1 + std::max(C.height, E.height);
      B.height = 1 + std::max(A.height, D.height);
    } else {
      B.child2 = iE;
      A.child1 = iD;
      D.parent = iA;
      A.fatAABB = unionOf(C.fatAABB, D.fatAABB);
      B.fatAABB = unionOf(A.fatAABB, E.fatAABB);
      A.height = 1 + std::max(C.height, D.height);
      B.height = 1 + std::max(A.height, E.height);
    }

    return iB;
  }

  return iA;
}
