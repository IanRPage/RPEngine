#ifndef QUADTREE_H
#define QUADTREE_H

#include <dsa/AABB.hpp>
#include <memory>
#include <vector>

template <typename T>
class QuadTree {
 private:
  size_t capacity_;
  std::vector<T*> data_;
  AABBf boundary_;
  bool divided_ = false;

  std::unique_ptr<QuadTree> ul_, ur_, bl_, br_;

  void subdivide() {
    const float x = boundary_.min.x, y = boundary_.min.y;
    const float w = 0.5f * boundary_.width(), h = 0.5f * boundary_.height();

    ul_ = std::make_unique<QuadTree>(AABBf({x, y}, {w, h}), capacity_);
    ur_ = std::make_unique<QuadTree>(AABBf({x + w, y}, {w, h}), capacity_);
    bl_ = std::make_unique<QuadTree>(AABBf({x, y + h}, {w, h}), capacity_);
    br_ = std::make_unique<QuadTree>(AABBf({x + w, y + h}, {w, h}), capacity_);

    divided_ = true;

    std::vector<T*> old = std::move(data_);
    for (T* p : old) {
      insert(p);
    }
  };

 public:
  QuadTree(AABBf bound, size_t cap) : capacity_(cap), boundary_(bound) {
    data_.reserve(capacity_);
  };

  bool insert(T* p) {
    const Vec2f pos(p->position.x, p->position.y);
    if (!boundary_.contains(pos)) {
      return false;
    }

    if (!divided_ && data_.size() < capacity_) {
      data_.push_back(p);
      return true;
    }

    if (!divided_) {
      subdivide();
    }

    if (ul_->insert(p)) return true;
    if (ur_->insert(p)) return true;
    if (bl_->insert(p)) return true;
    if (br_->insert(p)) return true;
    return false;
  };

  void query(std::vector<T*>& res, const AABBf& qRange) const {
    if (!boundary_.intersects(qRange)) return;

    for (T* p : data_) {
      const Vec2f pos(p->position.x, p->position.y);
      if (qRange.contains(pos)) res.push_back(p);
    }

    if (divided_) {
      if (ul_) ul_->query(res, qRange);
      if (ur_) ur_->query(res, qRange);
      if (bl_) bl_->query(res, qRange);
      if (br_) br_->query(res, qRange);
    }
  };
};

#endif
