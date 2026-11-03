#ifndef RPENGINE_CORE_WORLD_HPP
#define RPENGINE_CORE_WORLD_HPP

#include <broadphase/IBroadphase.hpp>
#include <collision/Epa.hpp>
#include <collision/Gjk.hpp>
#include <collision/ManifoldCache.hpp>
#include <core/BodyStore.hpp>
#include <dynamics/Solver.hpp>
#include <functional>
#include <memory>
#include <vector>

class World {
 public:
  World();
  explicit World(std::unique_ptr<IBroadphase> broadphase);

  BodyHandle createDynamicBody(const ShapeVariant& shape,
                               const Transform& transform, float mass,
                               float friction, float restitution,
                               bool constrainTo2D = false) noexcept;

  BodyHandle createStaticBody(const ShapeVariant& shape,
                              const Transform& transform, float friction,
                              float restitution,
                              bool constrainTo2D = false) noexcept;

  void addWorldBoundaries(Vec3f worldMin, Vec3f worldMax, float thickness,
                          float friction, float restitution,
                          bool is2D) noexcept;

  void step(float dt) noexcept;

  void snapshotPrevState() noexcept;

  void wakeAll() noexcept { bodies_.wakeAll(); }
  size_t sleepingBodyCount() const noexcept { return bodies_.sleepingCount(); }

  BodyStore& bodies() noexcept { return bodies_; }
  const BodyStore& bodies() const noexcept { return bodies_; }

  SolverConfig& config() noexcept { return config_; }
  const SolverConfig& config() const noexcept { return config_; }

  Vec3f gravity() const noexcept { return gravity_; }
  void setGravity(Vec3f gravity) noexcept { gravity_ = gravity; }

  void setBroadphase(std::unique_ptr<IBroadphase> broadphase) noexcept;

  using GjkFn = std::function<GjkResult(const ShapeVariant&, const Transform&,
                                        const ShapeVariant&, const Transform&)>;
  using EpaFn = std::function<EpaResult(const ShapeVariant&, const Transform&,
                                        const ShapeVariant&, const Transform&,
                                        const GjkResult&)>;
  void setNarrowphaseFns(GjkFn gjkFn, EpaFn epaFn) noexcept {
    if (gjkFn) { gjkFn_ = std::move(gjkFn); }
    if (epaFn) { epaFn_ = std::move(epaFn); }
  }

 private:
  void collideLeafPair(BodyHandle a, const ShapeVariant& leafA,
                       const Transform& ta, uint32_t idxA,
                       const Transform* localA, BodyHandle b,
                       const ShapeVariant& leafB, const Transform& tb,
                       uint32_t idxB, const Transform* localB) noexcept;
  void collideCompoundPair(BodyHandle a, const ShapeVariant& shapeA,
                           const Transform& ta, BodyHandle b,
                           const ShapeVariant& shapeB,
                           const Transform& tb) noexcept;

  struct ChildProxy {
    Transform transform;
    AABB aabb;
  };

  BodyStore bodies_;
  std::unique_ptr<IBroadphase> broadphase_;
  ManifoldCache manifoldCache_;
  SolverConfig config_;
  Vec3f gravity_{0.0f, -9.81f, 0.0f};

  GjkFn gjkFn_ = gjkOverlap;
  EpaFn epaFn_ = epaPenetration;

  std::vector<ChildProxy> childScratchB_;

  bool isAwakeDynamic(BodyHandle h) const noexcept;
  void wakeBodiesTouchingRemoved() noexcept;
  void propagateWakeThroughContacts(
      Vec3f wakeDeltaVelocity = Vec3f(0.0f)) noexcept;
  void updateSleep(std::span<const Manifold> manifolds, float dt) noexcept;

  uint32_t findRoot(uint32_t i) noexcept;
  void unite(uint32_t a, uint32_t b) noexcept;
  void resetUnionFind() noexcept;
  std::vector<uint32_t> ufParent_;
  std::vector<float> ufMinTimer_;
  std::vector<uint8_t> ufAwake_;

  static constexpr float kManifoldMatchFactor = 0.05f;
};

#endif
