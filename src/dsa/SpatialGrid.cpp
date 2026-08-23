#include <algorithm>
#include <dsa/SpatialGrid.hpp>

void SpatialGrid::configure(float cellSize, Vec2f worldSize) noexcept {
  invCellSize = 1.0f / cellSize;
  cols = worldSize.x * invCellSize;
  rows = worldSize.y * invCellSize;
  nCells = cols * rows;
}

void SpatialGrid::resize(size_t numItems) noexcept {
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
