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

  void configure(float cellSize, Vec2f worldSize) noexcept;
  void resize(size_t numItems) noexcept;

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
  inline void queryDoSomething(size_t objIdx, const float radius,
                               const Vec2f& pos, Fn&& callback) {
    int cxMin = std::clamp((int)((pos.x - radius) * invCellSize), 0, cols - 1);
    int cxMax = std::clamp((int)((pos.x + radius) * invCellSize), 0, cols - 1);
    int cyMin = std::clamp((int)((pos.y - radius) * invCellSize), 0, rows - 1);
    int cyMax = std::clamp((int)((pos.y + radius) * invCellSize), 0, rows - 1);

    // query neighbors
    for (int cy = cyMin; cy <= cyMax; cy++) {
      for (int cx = cxMin; cx <= cxMax; cx++) {
        const int cn = cy * cols + cx;
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
