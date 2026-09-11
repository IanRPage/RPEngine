#ifndef RPENGINE_CORE_BODYSTORE_HPP
#define RPENGINE_CORE_BODYSTORE_HPP

#include <collision/AABB.hpp>
#include <collision/Shapes.hpp>
#include <core/BodyHandle.hpp>
#include <math/Transform.hpp>
#include <math/Types.hpp>
#include <vector>

struct BodyDesc {
  ShapeVariant shape;
  Transform transform;
  float invMass = 0.0f;
  Mat3f invInertiaBody{0.0f};
  float friction = 0.5f;
  float restitution = 0.0f;
  bool constrainTo2D = false;
};

class BodyStore {
 public:
  BodyHandle addBody(const AABB& realAABB,
                     Vec3f displacement = Vec3f(0.0f)) noexcept;

  BodyHandle addBody(const BodyDesc& desc) noexcept;

  void removeBody(BodyHandle handle) noexcept;

  void setAABB(BodyHandle handle, const AABB& realAABB,
               Vec3f displacement) noexcept;

  bool isLive(BodyHandle handle) const noexcept;
  const AABB& aabb(BodyHandle handle) const noexcept;
  Vec3f displacement(BodyHandle handle) const noexcept;

  const ShapeVariant& shape(BodyHandle handle) const noexcept;

  Vec3f& position(BodyHandle handle) noexcept;
  Vec3f position(BodyHandle handle) const noexcept;
  Vec3f& prevPosition(BodyHandle handle) noexcept;
  Vec3f prevPosition(BodyHandle handle) const noexcept;

  Quatf& orientation(BodyHandle handle) noexcept;
  Quatf orientation(BodyHandle handle) const noexcept;
  Quatf& prevOrientation(BodyHandle handle) noexcept;
  Quatf prevOrientation(BodyHandle handle) const noexcept;

  Vec3f& linearVelocity(BodyHandle handle) noexcept;
  Vec3f linearVelocity(BodyHandle handle) const noexcept;
  Vec3f& angularVelocity(BodyHandle handle) noexcept;
  Vec3f angularVelocity(BodyHandle handle) const noexcept;

  float invMass(BodyHandle handle) const noexcept;
  const Mat3f& invInertiaBody(BodyHandle handle) const noexcept;
  float friction(BodyHandle handle) const noexcept;
  float restitution(BodyHandle handle) const noexcept;
  bool constrainTo2D(BodyHandle handle) const noexcept;

  Transform transform(BodyHandle handle) const noexcept;

  const std::vector<BodyHandle>& liveHandles() const noexcept {
    return liveHandles_;
  }
  size_t size() const noexcept { return liveHandles_.size(); }

 private:
  struct Slot {
    AABB aabb{Vec3f(0.0f), Vec3f(0.0f)};
    Vec3f displacement{0.0f};

    ShapeVariant shape{SphereShape{0.0f}};
    Vec3f position{0.0f};
    Vec3f prevPosition{0.0f};
    Quatf orientation{1.0f, 0.0f, 0.0f, 0.0f};
    Quatf prevOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3f linearVelocity{0.0f};
    Vec3f angularVelocity{0.0f};
    float invMass = 0.0f;
    Mat3f invInertiaBody{0.0f};
    float friction = 0.5f;
    float restitution = 0.0f;
    bool constrainTo2D = false;

    uint32_t generation = 0;
    bool alive = false;
  };

  uint32_t allocateSlot() noexcept;

  std::vector<Slot> slots_;
  std::vector<uint32_t> freeList_;
  std::vector<BodyHandle> liveHandles_;

  // index of `handle` in liveHandles_. makes so removeBody can swap-erase in
  // O(1) w/o invalidating any other live handle
  std::vector<size_t> liveIndex_;
};

#endif
