#ifndef SPATIALHASHMAP_H
#define SPATIALHASHMAP_H

#include <dsa/Vec2.hpp>
#include <cstdint>
#include <unordered_map>
#include <vector>

template <typename T>
class SpatialHashMap {
 public:
  SpatialHashMap(float cellSize, int capacity)
      : cellSize_(cellSize), invCellSize_(1.0f / cellSize) {
    map_.reserve(capacity);
  };

  void insert(T* obj, const Vec2f& pos) {
    const int x = static_cast<int>(pos.x * invCellSize_);
    const int y = static_cast<int>(pos.y * invCellSize_);
    const uint64_t hash = easyHash(x, y);
    map_[hash].push_back(obj);
  };

  void queryNeighbors(std::vector<T*>& neighbors, Vec2f& pos) const {
    neighbors.clear();

    const int cellX = static_cast<int>(pos.x * invCellSize_);
    const int cellY = static_cast<int>(pos.y * invCellSize_);

    for (int dx = -1; dx < 2; dx++) {
      for (int dy = -1; dy < 2; dy++) {
        const uint64_t hash = easyHash(cellX + dx, cellY + dy);
        auto it = map_.find(hash);
        if (it != map_.end()) {
          const std::vector<T*> bucket = it->second;
          neighbors.insert(neighbors.end(), bucket.begin(), bucket.end());
        }
      }
    }
  };

  void clear() { map_.clear(); };

 private:
  float cellSize_;
  float invCellSize_;
  std::unordered_map<uint64_t, std::vector<T*>> map_;

  // receives grid coordinates, returns hash
  static uint64_t easyHash(int x, int y) {
    uint64_t ux = static_cast<uint64_t>(x);
    uint64_t uy = static_cast<uint64_t>(y);
    return (ux << 32) | uy;
  }
};

#endif
