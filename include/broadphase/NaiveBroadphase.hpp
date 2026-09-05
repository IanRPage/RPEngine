#ifndef RPENGINE_BROADPHASE_NAIVEBROADPHASE_HPP
#define RPENGINE_BROADPHASE_NAIVEBROADPHASE_HPP

#include <broadphase/IBroadphase.hpp>
#include <span>
#include <utility>
#include <vector>

class NaiveBroadphase final : public IBroadphase {
 public:
  std::span<const std::pair<BodyHandle, BodyHandle>> computePairs(
      const BodyStore& bodies) override;

 private:
  std::vector<std::pair<BodyHandle, BodyHandle>> pairs_;
};

#endif
