#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <algorithm>
#include <cstddef>
#include <dsa/Vec2.hpp>
#include <vector>

struct SpatialGrid {
  float invCellSize;
  int cols, rows, nCells;

  std::vector<int> head, next;
  size_t headSize = 0, nextSize = 0;

  inline void configure(float cellSize, Vec2f worldSize) noexcept {
    invCellSize = 1.0f / cellSize;
    cols = worldSize.x * invCellSize;
    rows = worldSize.y * invCellSize;
    nCells = cols * rows;
  };

  inline void resize(size_t numItems) noexcept {
    if (headSize != static_cast<size_t>(nCells)) {
      head.assign(nCells, -1);
      headSize = nCells;
    } else {
      std::fill(head.begin(), head.end(), -1);
    }

    if (nextSize != numItems) {
      nextSize = numItems;
      next.resize(nextSize);
    }
  }

  template <typename T>
  inline void build(std::vector<T>& items) noexcept {
    for (size_t i = 0; i < items.size(); i++) {
      const Vec2f& pos = items[i].position;
      int cx = static_cast<int>(pos.x * invCellSize);
      int cy = static_cast<int>(pos.y * invCellSize);
      cx = std::clamp(cx, 0, cols - 1);
      cy = std::clamp(cy, 0, rows - 1);

      // c maps grid coords to flat index in head
      const int c = cy * cols + cx;

      // push_front operation
      next[i] = head[c];
      head[c] = i;
    }
  }
};

#endif
