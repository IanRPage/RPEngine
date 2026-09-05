#ifndef RPENGINE_CORE_BODYSTORE_HPP
#define RPENGINE_CORE_BODYSTORE_HPP

#include <collision/AABB.hpp>
#include <core/BodyHandle.hpp>
#include <math/Types.hpp>
#include <vector>

class BodyStore {
 public:
  BodyHandle addBody(const AABB& realAABB,
                     Vec3f displacement = Vec3f(0.0f)) noexcept;
  void removeBody(BodyHandle handle) noexcept;

  void setAABB(BodyHandle handle, const AABB& realAABB,
               Vec3f displacement) noexcept;

  bool isLive(BodyHandle handle) const noexcept;
  const AABB& aabb(BodyHandle handle) const noexcept;
  Vec3f displacement(BodyHandle handle) const noexcept;

  const std::vector<BodyHandle>& liveHandles() const noexcept {
    return liveHandles_;
  }
  size_t size() const noexcept { return liveHandles_.size(); }

 private:
  struct Slot {
    AABB aabb{Vec3f(0.0f), Vec3f(0.0f)};
    Vec3f displacement{0.0f};
    uint32_t generation = 0;
    bool alive = false;
  };

  std::vector<Slot> slots_;
  std::vector<uint32_t> freeList_;
  std::vector<BodyHandle> liveHandles_;

  // index of `handle` in liveHandles_. makes so removeBody can swap-erase in
  // O(1) w/o invalidating any other live handle
  std::vector<size_t> liveIndex_;
};

#endif
