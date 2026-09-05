#include <core/BodyStore.hpp>

BodyHandle BodyStore::addBody(const AABB& realAABB,
                              Vec3f displacement) noexcept {
  uint32_t index;
  if (!freeList_.empty()) {
    index = freeList_.back();
    freeList_.pop_back();
  } else {
    index = static_cast<uint32_t>(slots_.size());
    slots_.emplace_back();
  }

  Slot& slot = slots_[index];
  slot.aabb = realAABB;
  slot.displacement = displacement;
  slot.alive = true;

  BodyHandle handle{index, slot.generation};

  if (liveIndex_.size() <= index) liveIndex_.resize(index + 1);
  liveIndex_[index] = liveHandles_.size();
  liveHandles_.push_back(handle);

  return handle;
}

void BodyStore::removeBody(BodyHandle handle) noexcept {
  if (!isLive(handle)) return;

  Slot& slot = slots_[handle.index];
  slot.alive = false;
  slot.generation++;

  const size_t removedAt = liveIndex_[handle.index];
  const size_t lastAt = liveHandles_.size() - 1;
  if (removedAt != lastAt) {
    liveHandles_[removedAt] = liveHandles_[lastAt];
    liveIndex_[liveHandles_[removedAt].index] = removedAt;
  }
  liveHandles_.pop_back();

  freeList_.push_back(handle.index);
}

void BodyStore::setAABB(BodyHandle handle, const AABB& realAABB,
                        Vec3f displacement) noexcept {
  if (!isLive(handle)) return;
  Slot& slot = slots_[handle.index];
  slot.aabb = realAABB;
  slot.displacement = displacement;
}

bool BodyStore::isLive(BodyHandle handle) const noexcept {
  return handle.valid() && handle.index < slots_.size() &&
         slots_[handle.index].alive &&
         slots_[handle.index].generation == handle.generation;
}

const AABB& BodyStore::aabb(BodyHandle handle) const noexcept {
  return slots_[handle.index].aabb;
}

Vec3f BodyStore::displacement(BodyHandle handle) const noexcept {
  return slots_[handle.index].displacement;
}
