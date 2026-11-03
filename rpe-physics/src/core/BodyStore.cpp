#include <core/BodyStore.hpp>

uint32_t BodyStore::allocateSlot() noexcept {
  uint32_t index;
  if (!freeList_.empty()) {
    index = freeList_.back();
    freeList_.pop_back();
  } else {
    index = static_cast<uint32_t>(slots_.size());
    slots_.emplace_back();
  }

  Slot& slot = slots_[index];
  uint32_t generation = slot.generation;
  slot = Slot{};
  slot.generation = generation;
  slot.alive = true;

  BodyHandle handle{index, slot.generation};
  if (liveIndex_.size() <= index) { liveIndex_.resize(index + 1); }
  liveIndex_[index] = liveHandles_.size();
  liveHandles_.push_back(handle);

  return index;
}

BodyHandle BodyStore::addBody(const AABB& realAABB,
                              Vec3f displacement) noexcept {
  uint32_t index = allocateSlot();
  Slot& slot = slots_[index];
  slot.aabb = realAABB;
  slot.displacement = displacement;
  return BodyHandle{index, slot.generation};
}

BodyHandle BodyStore::addBody(const BodyDesc& desc) noexcept {
  uint32_t index = allocateSlot();
  Slot& slot = slots_[index];
  slot.shape = desc.shape;
  slot.position = desc.transform.position;
  slot.prevPosition = desc.transform.position;
  slot.orientation = desc.transform.orientation;
  slot.prevOrientation = desc.transform.orientation;
  slot.invMass = desc.invMass;
  slot.invInertiaBody = desc.invInertiaBody;
  slot.friction = desc.friction;
  slot.restitution = desc.restitution;
  slot.constrainTo2D = desc.constrainTo2D;
  slot.aabb = worldAABB(slot.shape, desc.transform);
  slot.displacement = Vec3f(0.0f);
  return BodyHandle{index, slot.generation};
}

void BodyStore::removeBody(BodyHandle handle) noexcept {
  if (!isLive(handle)) { return; }

  if (slots_[handle.index].sleeping) { wake(handle); }

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
  if (!isLive(handle)) { return; }
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

const ShapeVariant& BodyStore::shape(BodyHandle handle) const noexcept {
  return slots_[handle.index].shape;
}

Vec3f& BodyStore::position(BodyHandle handle) noexcept {
  return slots_[handle.index].position;
}

Vec3f BodyStore::position(BodyHandle handle) const noexcept {
  return slots_[handle.index].position;
}

Vec3f& BodyStore::prevPosition(BodyHandle handle) noexcept {
  return slots_[handle.index].prevPosition;
}

Vec3f BodyStore::prevPosition(BodyHandle handle) const noexcept {
  return slots_[handle.index].prevPosition;
}

Quatf& BodyStore::orientation(BodyHandle handle) noexcept {
  return slots_[handle.index].orientation;
}

Quatf BodyStore::orientation(BodyHandle handle) const noexcept {
  return slots_[handle.index].orientation;
}

Quatf& BodyStore::prevOrientation(BodyHandle handle) noexcept {
  return slots_[handle.index].prevOrientation;
}

Quatf BodyStore::prevOrientation(BodyHandle handle) const noexcept {
  return slots_[handle.index].prevOrientation;
}

Vec3f& BodyStore::linearVelocity(BodyHandle handle) noexcept {
  return slots_[handle.index].linearVelocity;
}

Vec3f BodyStore::linearVelocity(BodyHandle handle) const noexcept {
  return slots_[handle.index].linearVelocity;
}

Vec3f& BodyStore::angularVelocity(BodyHandle handle) noexcept {
  return slots_[handle.index].angularVelocity;
}

Vec3f BodyStore::angularVelocity(BodyHandle handle) const noexcept {
  return slots_[handle.index].angularVelocity;
}

float BodyStore::invMass(BodyHandle handle) const noexcept {
  return slots_[handle.index].invMass;
}

const Mat3f& BodyStore::invInertiaBody(BodyHandle handle) const noexcept {
  return slots_[handle.index].invInertiaBody;
}

float BodyStore::friction(BodyHandle handle) const noexcept {
  return slots_[handle.index].friction;
}

float BodyStore::restitution(BodyHandle handle) const noexcept {
  return slots_[handle.index].restitution;
}

bool BodyStore::constrainTo2D(BodyHandle handle) const noexcept {
  return slots_[handle.index].constrainTo2D;
}

Transform BodyStore::transform(BodyHandle handle) const noexcept {
  const Slot& slot = slots_[handle.index];
  return Transform{slot.position, slot.orientation};
}

bool BodyStore::isSleeping(BodyHandle handle) const noexcept {
  return isLive(handle) && slots_[handle.index].sleeping;
}

float& BodyStore::sleepTimer(BodyHandle handle) noexcept {
  return slots_[handle.index].sleepTimer;
}

float BodyStore::sleepTimer(BodyHandle handle) const noexcept {
  return slots_[handle.index].sleepTimer;
}

void BodyStore::sleepSlot(BodyHandle handle, uint32_t islandId) noexcept {
  Slot& slot = slots_[handle.index];
  if (!slot.sleeping) { sleepingCount_++; }
  slot.sleeping = true;
  slot.sleepIsland = islandId;
  slot.linearVelocity = Vec3f(0.0f);
  slot.angularVelocity = Vec3f(0.0f);
  slot.prevPosition = slot.position;
  slot.prevOrientation = slot.orientation;
}

void BodyStore::wakeSlot(Slot& slot) noexcept {
  slot.sleepTimer = 0.0f;
  if (!slot.sleeping) { return; }
  slot.sleeping = false;
  slot.sleepIsland = 0;
  sleepingCount_--;
}

void BodyStore::putToSleep(BodyHandle handle) noexcept {
  putIslandToSleep(std::span<const BodyHandle>(&handle, 1));
}

void BodyStore::putIslandToSleep(std::span<const BodyHandle> island) noexcept {
  uint32_t islandId = nextIslandId_++;
  if (islandId == 0) { islandId = nextIslandId_++; }

  std::vector<BodyHandle> members;
  members.reserve(island.size());
  for (BodyHandle h : island) {
    if (!isLive(h) || slots_[h.index].invMass <= 0.0f) { continue; }
    sleepSlot(h, islandId);
    members.push_back(h);
  }
  if (!members.empty()) { sleepingIslands_[islandId] = std::move(members); }
}

void BodyStore::wake(BodyHandle handle) noexcept {
  if (!isLive(handle)) { return; }
  Slot& slot = slots_[handle.index];
  if (!slot.sleeping) {
    slot.sleepTimer = 0.0f;
    return;
  }

  const uint32_t islandId = slot.sleepIsland;
  wakeSlot(slot);

  auto it = sleepingIslands_.find(islandId);
  if (it == sleepingIslands_.end()) { return; }
  for (BodyHandle member : it->second) {
    if (!isLive(member)) { continue; }
    Slot& m = slots_[member.index];
    if (m.sleeping && m.sleepIsland == islandId) { wakeSlot(m); }
  }
  sleepingIslands_.erase(it);
}

void BodyStore::wakeAll() noexcept {
  for (BodyHandle h : liveHandles_) { wakeSlot(slots_[h.index]); }
  sleepingIslands_.clear();
}

size_t BodyStore::sleepingCount() const noexcept { return sleepingCount_; }
