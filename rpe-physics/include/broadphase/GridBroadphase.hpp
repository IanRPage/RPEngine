#ifndef RPENGINE_BROADPHASE_GRIDBROADPHASE_HPP
#define RPENGINE_BROADPHASE_GRIDBROADPHASE_HPP

#include <broadphase/IBroadphase.hpp>
#include <cstdint>
#include <math/Types.hpp>
#include <span>
#include <utility>
#include <vector>

class GridBroadphase final : public IBroadphase {
 public:
  explicit GridBroadphase(float cellSize = 20.0f) noexcept;

  std::span<const std::pair<BodyHandle, BodyHandle>> computePairs(
      const BodyStore& bodies) override;

 private:
  void configureFor(const BodyStore& bodies) noexcept;
  void build(const BodyStore& bodies) noexcept;
  int cellIndex(Vec3f center) const noexcept;

  float cellSize_;
  float invCellSize_ = 1.0f;
  Vec3f worldMin_{0.0f};
  int cols_ = 1, rows_ = 1, depth_ = 1, nCells_ = 1;

  float maxRadius_ = 0.0f;

  std::vector<int32_t> head_;
  std::vector<int32_t> next_;

  std::vector<std::pair<BodyHandle, BodyHandle>> pairs_;
};

#endif
