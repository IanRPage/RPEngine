#ifndef QUADTREE_H
#define QUADTREE_H

#include <dsa/AABB.hpp>
#include <vector>

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

  bool insert(size_t i, const Vec2f& pos) {
    if (!boundary_.contains(pos)) {
      return false;
    }

    if (!divided_ && data_.size() < capacity_) {
      data_.push_back(i);
      return true;
    }

    if (!divided_) {
      subdivide([&](size_t) { return Vec2f(); });
    }

    if (ul_ && ul_->insert(i, pos)) return true;
    if (ur_ && ur_->insert(i, pos)) return true;
    if (bl_ && bl_->insert(i, pos)) return true;
    if (br_ && br_->insert(i, pos)) return true;
    return false;
  };

  template <typename Fn>
  void query(std::vector<size_t>& res, const AABBf& qRange,
             Fn&& callback) const {
    if (!boundary_.intersects(qRange)) return;

    for (size_t i : data_) {
      const Vec2f pos = callback(i);
      if (qRange.contains(pos)) res.push_back(i);
    }

    if (divided_) {
      if (ul_) ul_->query(res, qRange, callback);
      if (ur_) ur_->query(res, qRange, callback);
      if (bl_) bl_->query(res, qRange, callback);
      if (br_) br_->query(res, qRange, callback);
    }
  };

 private:
  size_t capacity_;
  std::vector<size_t> data_;
  AABBf boundary_;
  bool divided_ = false;

  QuadTree *ul_, *ur_, *bl_, *br_;

  template <typename Fn>
  void subdivide(Fn&& callback) {
    const float x = boundary_.min.x, y = boundary_.min.y;
    const float w = 0.5f * boundary_.width(), h = 0.5f * boundary_.height();

    ul_ = new QuadTree(AABBf({x, y}, {w, h}), capacity_);
    ur_ = new QuadTree(AABBf({x + w, y}, {w, h}), capacity_);
    bl_ = new QuadTree(AABBf({x, y + h}, {w, h}), capacity_);
    br_ = new QuadTree(AABBf({x + w, y + h}, {w, h}), capacity_);

    divided_ = true;

    std::vector<size_t> old = std::move(data_);
    for (size_t i : old) {
      insert(i, callback(i));
    }
  };
};

#endif
