#ifndef QUADTREE_H
#define QUADTREE_H

#include <dsa/AABB.hpp>
#include <vector>

template <typename T>
class QuadTree {
 public:
  QuadTree(AABBf bound, size_t cap)
      : capacity_(cap),
        boundary_(bound),
        ul_(nullptr),
        ur_(nullptr),
        bl_(nullptr),
        br_(nullptr) {
    data_.reserve(cap);
  };
  ~QuadTree() {
    delete ul_;
    delete ur_;
    delete bl_;
    delete br_;
  }

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

    if (ul_ && ul_->insert(p)) return true;
    if (ur_ && ur_->insert(p)) return true;
    if (bl_ && bl_->insert(p)) return true;
    if (br_ && br_->insert(p)) return true;
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

 private:
  size_t capacity_;
  std::vector<T*> data_;
  AABBf boundary_;
  bool divided_ = false;

  QuadTree *ul_, *ur_, *bl_, *br_;

  void subdivide() {
    const float x = boundary_.min.x, y = boundary_.min.y;
    const float w = 0.5f * boundary_.width(), h = 0.5f * boundary_.height();

    ul_ = new QuadTree(AABBf({x, y}, {w, h}), capacity_);
    ur_ = new QuadTree(AABBf({x + w, y}, {w, h}), capacity_);
    bl_ = new QuadTree(AABBf({x, y + h}, {w, h}), capacity_);
    br_ = new QuadTree(AABBf({x + w, y + h}, {w, h}), capacity_);

    divided_ = true;

    std::vector<T*> old = std::move(data_);
    for (T* p : old) {
      insert(p);
    }
  };
};

#endif
