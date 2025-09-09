#ifndef QUADTREE_H
#define QUADTREE_H

#include <dsa/AABB.h>
#include <memory>
#include <vector>

template <typename T> class QuadTree {
private:
  size_t capacity;
  std::vector<T *> data;
  AABBf boundary;
  bool divided = false;

  std::unique_ptr<QuadTree> ul, ur, bl, br;

  void subdivide() {
    const float x = boundary.min.x, y = boundary.min.y;
    const float hw = 0.5f * boundary.width(), hh = 0.5f * boundary.height();

    ul = std::make_unique<QuadTree>(AABBf({x, y}, {hw, hh}), capacity);
    ur = std::make_unique<QuadTree>(AABBf({x + hw, y}, {hw, hh}), capacity);
    bl = std::make_unique<QuadTree>(AABBf({x, y + hh}, {hw, hh}), capacity);
    br = std::make_unique<QuadTree>(AABBf({x + hw, y + hh}, {hw, hh}), capacity);

    divided = true;

    std::vector<T *> old = std::move(data);
    for (T *p : old) {
      insert(p);
    }
  };

public:
  QuadTree(AABBf bound, size_t cap) : capacity(cap), boundary(bound) {
    data.reserve(capacity);
  };

  bool insert(T *p) {
    const Vec2f pos(p->position.x, p->position.y);
    if (!boundary.contains(pos)) {
      return false;
    }

    if (!divided && data.size() < capacity) {
      data.push_back(p);
      return true;
    }

    if (!divided) {
      subdivide();
    }

    if (ul->insert(p)) return true;
    if (ur->insert(p)) return true;
    if (bl->insert(p)) return true;
    if (br->insert(p)) return true;
    return false;
  };

  void query(std::vector<T *> &res, const AABBf &qRange) const {
    if (!boundary.intersects(qRange)) return;

    for (T *p : data) {
      const Vec2f pos(p->position.x, p->position.y);
      if (qRange.contains(pos))
				res.push_back(p);
    }

    if (divided) {
      if (ul) ul->query(res, qRange);
      if (ur) ur->query(res, qRange);
      if (bl) bl->query(res, qRange);
      if (br) br->query(res, qRange);
    }
  };
};

#endif
