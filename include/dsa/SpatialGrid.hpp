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

  template <typename Fn>
  inline void queryDoSomething(size_t objIdx, const Vec2f& pos, Fn&& callback,
                               bool reverse = false, int scale = 1) {
    const int cx =
        std::clamp(static_cast<int>(pos.x * invCellSize), 0, cols - 1);
    const int cy =
        std::clamp(static_cast<int>(pos.y * invCellSize), 0, rows - 1);

    // precompute valid neighbor ranges
    int dxMin = 0, dxMax = 0, dyMin = 0, dyMax = 0;
    if (scale != 0) {
      dxMin = -std::min(scale, cx);
      dxMax = std::min(scale, cols - 1 - cx);
      dyMin = -std::min(scale, cy);
      dyMax = std::min(scale, rows - 1 - cy);
    } else {
      return;
    }

    const int dxStart = reverse ? dxMax : dxMin;
    const int dxEnd = reverse ? dxMin : dxMax;
    const int dxStep = reverse ? -1 : 1;
    const int dyStart = reverse ? dyMax : dyMin;
    const int dyEnd = reverse ? dyMin : dyMax;
    const int dyStep = reverse ? -1 : 1;

    // query neighbors
    for (int dx = dxStart; reverse ? (dx >= dxEnd) : (dx <= dxEnd);
         dx += dxStep) {
      const int nx = cx + dx;
      for (int dy = dyStart; reverse ? (dy >= dyEnd) : (dy <= dyEnd);
           dy += dyStep) {
        const int ny = cy + dy;
        const int cn = ny * cols + nx;
        if (cn >= static_cast<int>(head.size())) continue;

        // get the second particle
        for (int idx = head[cn]; idx != -1; idx = next[idx]) {
          // prune redundant checks
          if (idx <= static_cast<int>(objIdx)) continue;
          callback(idx);
        }
      }
    }
  };
};

#endif
