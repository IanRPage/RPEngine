#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <algorithm>
#include <cstddef>
#include <dsa/Vec2.hpp>
#include <vector>

struct SpatialGrid {
  float invCellSize_;
  int cols_, rows_, nCells_;

  std::vector<int> head_;
  std::vector<int> next_;
  size_t headSize_ = 0, nextSize_ = 0;

  inline void configure(float cellSize, Vec2f worldSize) noexcept {
    invCellSize_ = 1.0f / cellSize;
    cols_ = worldSize.x * invCellSize_;
    rows_ = worldSize.y * invCellSize_;
    nCells_ = cols_ * rows_;
  };

  inline void resize(size_t numItems) noexcept {
    if (headSize_ != static_cast<size_t>(nCells_)) {
      head_.assign(nCells_, -1);
      headSize_ = nCells_;
    } else {
      std::fill(head_.begin(), head_.end(), -1);
    }

    if (nextSize_ != numItems) {
      nextSize_ = numItems;
      next_.resize(nextSize_);
    }
  }

  template <typename T>
  inline void build(std::vector<T>& items) noexcept {
    for (size_t i = 0; i < items.size(); i++) {
      const Vec2f& pos = items[i].position;
      int cx = static_cast<int>(pos.x * invCellSize_);
      int cy = static_cast<int>(pos.y * invCellSize_);
      cx = std::clamp(cx, 0, cols_ - 1);
      cy = std::clamp(cy, 0, rows_ - 1);

      // c maps grid coords to flat index in head
      const int c = cy * cols_ + cx;

      // push_front operation
      next_[i] = head_[c];
      head_[c] = i;
    }
  }
};

#endif
