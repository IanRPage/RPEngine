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

  BodyStore& bodies() noexcept { return bodies_; }
  const BodyStore& bodies() const noexcept { return bodies_; }

  SolverConfig& config() noexcept { return config_; }
  const SolverConfig& config() const noexcept { return config_; }

  Vec3f gravity() const noexcept { return gravity_; }
  void setGravity(Vec3f gravity) noexcept { gravity_ = gravity; }

  void setBroadphase(std::unique_ptr<IBroadphase> broadphase) noexcept {
    broadphase_ = std::move(broadphase);
  }

  using GjkFn = std::function<GjkResult(const ShapeVariant&, const Transform&,
                                        const ShapeVariant&, const Transform&)>;
  using EpaFn = std::function<EpaResult(const ShapeVariant&, const Transform&,
                                        const ShapeVariant&, const Transform&,
                                        const GjkResult&)>;
  void setNarrowphaseFns(GjkFn gjkFn, EpaFn epaFn) noexcept {
    gjkFn_ = std::move(gjkFn);
    epaFn_ = std::move(epaFn);
  }

 private:
  BodyStore bodies_;
  std::unique_ptr<IBroadphase> broadphase_;
  ManifoldCache manifoldCache_;
  SolverConfig config_;
  Vec3f gravity_{0.0f, -9.81f, 0.0f};

  GjkFn gjkFn_ = gjkOverlap;
  EpaFn epaFn_ = epaPenetration;

  static constexpr float kManifoldMatchFactor = 0.05f;
};

#endif
