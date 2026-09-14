#ifndef RPENGINE_BROADPHASE_IBROADPHASE_HPP
#define RPENGINE_BROADPHASE_IBROADPHASE_HPP

#include <core/BodyHandle.hpp>
#include <core/BodyStore.hpp>
#include <span>
#include <utility>

class IBroadphase {
 public:
  virtual ~IBroadphase() = default;
  virtual std::span<const std::pair<BodyHandle, BodyHandle>> computePairs(
      const BodyStore& bodies) = 0;
};

#endif
