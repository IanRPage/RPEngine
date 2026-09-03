#include <algorithm>
#include <broadphase/GridBroadphase.hpp>
#include <core/BodyStore.hpp>

GridBroadphase::GridBroadphase(float cellSize) noexcept : cellSize_(cellSize) {}

int GridBroadphase::cellIndex(Vec3f center) const noexcept {
  Vec3f local = center - worldMin_;
  int cx = std::clamp(static_cast<int>(local.x * invCellSize_), 0, cols_ - 1);
  int cy = std::clamp(static_cast<int>(local.y * invCellSize_), 0, rows_ - 1);
  int cz = std::clamp(static_cast<int>(local.z * invCellSize_), 0, depth_ - 1);
  return (cz * rows_ + cy) * cols_ + cx;
}

void GridBroadphase::configureFor(const BodyStore& bodies) noexcept {
  invCellSize_ = 1.0f / cellSize_;

  const std::vector<BodyHandle>& handles = bodies.liveHandles();
  if (handles.empty()) {
    worldMin_ = Vec3f(0.0f);
    cols_ = rows_ = depth_ = 1;
    nCells_ = 1;
    return;
  }

  Vec3f mn = bodies.aabb(handles[0]).min;
  Vec3f mx = bodies.aabb(handles[0]).max;
  for (size_t i = 1; i < handles.size(); i++) {
    const AABB& a = bodies.aabb(handles[i]);
    mn = glm::min(mn, a.min);
    mx = glm::max(mx, a.max);
  }

  worldMin_ = mn;
  Vec3f extent = mx - mn;
  cols_ = std::max(1, static_cast<int>(extent.x * invCellSize_) + 1);
  rows_ = std::max(1, static_cast<int>(extent.y * invCellSize_) + 1);
  depth_ = std::max(1, static_cast<int>(extent.z * invCellSize_) + 1);
  nCells_ = cols_ * rows_ * depth_;
}

void GridBroadphase::build(const BodyStore& bodies) noexcept {
  head_.assign(static_cast<size_t>(nCells_), -1);

  const std::vector<BodyHandle>& handles = bodies.liveHandles();
  next_.assign(handles.size(), -1);

  for (size_t i = 0; i < handles.size(); i++) {
    const AABB& aabb = bodies.aabb(handles[i]);
    Vec3f center = (aabb.min + aabb.max) * 0.5f;
    int cell = cellIndex(center);
    next_[i] = head_[static_cast<size_t>(cell)];
    head_[static_cast<size_t>(cell)] = static_cast<int32_t>(i);
  }
}

std::span<const std::pair<BodyHandle, BodyHandle>> GridBroadphase::computePairs(
    const BodyStore& bodies) {
  pairs_.clear();
  configureFor(bodies);
  build(bodies);

  const std::vector<BodyHandle>& handles = bodies.liveHandles();

  for (size_t i = 0; i < handles.size(); i++) {
    const AABB& aabbI = bodies.aabb(handles[i]);
    Vec3f centerI = (aabbI.min + aabbI.max) * 0.5f;
    float radius = glm::length(aabbI.max - aabbI.min) * 0.5f;
    Vec3f local = centerI - worldMin_;

    int cxMin = std::clamp(static_cast<int>((local.x - radius) * invCellSize_),
                           0, cols_ - 1);
    int cxMax = std::clamp(static_cast<int>((local.x + radius) * invCellSize_),
                           0, cols_ - 1);
    int cyMin = std::clamp(static_cast<int>((local.y - radius) * invCellSize_),
                           0, rows_ - 1);
    int cyMax = std::clamp(static_cast<int>((local.y + radius) * invCellSize_),
                           0, rows_ - 1);
    int czMin = std::clamp(static_cast<int>((local.z - radius) * invCellSize_),
                           0, depth_ - 1);
    int czMax = std::clamp(static_cast<int>((local.z + radius) * invCellSize_),
                           0, depth_ - 1);

    for (int cz = czMin; cz <= czMax; cz++) {
      for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
          int cell = (cz * rows_ + cy) * cols_ + cx;
          for (int32_t j = head_[static_cast<size_t>(cell)]; j != -1;
               j = next_[static_cast<size_t>(j)]) {
            if (j <= static_cast<int32_t>(i))
              continue;  // prune self & redundant checks
            pairs_.emplace_back(handles[i], handles[static_cast<size_t>(j)]);
          }
        }
      }
    }
  }

  return pairs_;
}
