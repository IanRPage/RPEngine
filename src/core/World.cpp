#include <core/World.hpp>

#include <algorithm>
#include <broadphase/DynamicBVHBroadphase.hpp>
#include <collision/Manifold.hpp>
#include <collision/MassProperties.hpp>
#include <dynamics/Integrator.hpp>

namespace {
float boundingRadiusOf(const ShapeVariant& shape) noexcept {
  return std::visit([](const auto& s) { return s.boundingRadius(); }, shape);
}

std::unique_ptr<IBroadphase> normalizeBroadphase(
    std::unique_ptr<IBroadphase> broadphase) noexcept {
  return broadphase ? std::move(broadphase)
                    : std::make_unique<DynamicBVHBroadphase>();
}
}  // namespace

World::World() : World(std::make_unique<DynamicBVHBroadphase>()) {}

World::World(std::unique_ptr<IBroadphase> broadphase)
    : broadphase_(normalizeBroadphase(std::move(broadphase))) {}

void World::setBroadphase(std::unique_ptr<IBroadphase> broadphase) noexcept {
  broadphase_ = normalizeBroadphase(std::move(broadphase));
}

BodyHandle World::createDynamicBody(const ShapeVariant& shape,
                                    const Transform& transform, float mass,
                                    float friction, float restitution,
                                    bool constrainTo2D) noexcept {
  MassProperties mp = computeMassProperties(shape, mass);
  BodyDesc desc;
  desc.shape = shape;
  desc.transform = transform;
  desc.invMass = mp.invMass;
  desc.invInertiaBody = mp.invLocalInertiaTensor;
  desc.friction = friction;
  desc.restitution = restitution;
  desc.constrainTo2D = constrainTo2D;
  return bodies_.addBody(desc);
}

BodyHandle World::createStaticBody(const ShapeVariant& shape,
                                   const Transform& transform, float friction,
                                   float restitution,
                                   bool constrainTo2D) noexcept {
  BodyDesc desc;
  desc.shape = shape;
  desc.transform = transform;
  desc.invMass = 0.0f;
  desc.invInertiaBody = Mat3f(0.0f);
  desc.friction = friction;
  desc.restitution = restitution;
  desc.constrainTo2D = constrainTo2D;
  return bodies_.addBody(desc);
}

void World::addWorldBoundaries(Vec3f worldMin, Vec3f worldMax, float thickness,
                               float friction, float restitution,
                               bool is2D) noexcept {
  Vec3f center = (worldMin + worldMax) * 0.5f;
  Vec3f halfSize = (worldMax - worldMin) * 0.5f;
  float zHalf = is2D ? 0.0f : halfSize.z + thickness;
  float centerZ = is2D ? 0.0f : center.z;

  auto addWall = [&](Vec3f wallCenter, Vec3f wallHalfExtents) {
    Transform t;
    t.position = wallCenter;
    createStaticBody(ShapeVariant{BoxShape{wallHalfExtents}}, t, friction,
                     restitution, is2D);
  };

  addWall(Vec3f(worldMin.x - thickness, center.y, centerZ),
          Vec3f(thickness, halfSize.y + thickness, zHalf));
  addWall(Vec3f(worldMax.x + thickness, center.y, centerZ),
          Vec3f(thickness, halfSize.y + thickness, zHalf));
  addWall(Vec3f(center.x, worldMin.y - thickness, centerZ),
          Vec3f(halfSize.x + thickness, thickness, zHalf));
  addWall(Vec3f(center.x, worldMax.y + thickness, centerZ),
          Vec3f(halfSize.x + thickness, thickness, zHalf));

  if (!is2D) {
    addWall(Vec3f(center.x, center.y, worldMin.z - thickness),
            Vec3f(halfSize.x + thickness, halfSize.y + thickness, thickness));
    addWall(Vec3f(center.x, center.y, worldMax.z + thickness),
            Vec3f(halfSize.x + thickness, halfSize.y + thickness, thickness));
  }
}

void World::snapshotPrevState() noexcept {
  for (BodyHandle h : bodies_.liveHandles()) {
    bodies_.prevPosition(h) = bodies_.position(h);
    bodies_.prevOrientation(h) = bodies_.orientation(h);
  }
}

void World::step(float dt) noexcept {
  integrateVelocity(bodies_, dt, gravity_);

  for (BodyHandle h : bodies_.liveHandles()) {
    Transform t = bodies_.transform(h);
    AABB aabb = worldAABB(bodies_.shape(h), t);
    Vec3f displacement = bodies_.linearVelocity(h) * dt;
    bodies_.setAABB(h, aabb, displacement);
  }

  auto pairs = broadphase_->computePairs(bodies_);

  manifoldCache_.beginFrame();
  for (const auto& [a, b] : pairs) {
    const ShapeVariant& shapeA = bodies_.shape(a);
    const ShapeVariant& shapeB = bodies_.shape(b);
    Transform ta = bodies_.transform(a);
    Transform tb = bodies_.transform(b);

    GjkResult gjk = gjkFn_(shapeA, ta, shapeB, tb);
    if (!gjk.overlapping) { continue; }

    EpaResult epa = epaFn_(shapeA, ta, shapeB, tb, gjk);
    Manifold fresh = buildManifold(shapeA, ta, shapeB, tb, a, b, gjk, epa);

    float matchThreshold =
        kManifoldMatchFactor *
        std::min(boundingRadiusOf(shapeA), boundingRadiusOf(shapeB));
    manifoldCache_.updateManifold(fresh, matchThreshold);
  }
  manifoldCache_.endFrame();

  std::span<Manifold> activeManifolds = manifoldCache_.activeManifolds();

  std::vector<float> restitutionBias =
      prepareRestitutionBias(activeManifolds, bodies_);

  InvInertiaWorldCache invInertiaCache(bodies_);
  warmStart(activeManifolds, bodies_, invInertiaCache);
  for (int i = 0; i < config_.velocityIterations; i++) {
    solveVelocity(activeManifolds, bodies_, restitutionBias, invInertiaCache);
  }

  integratePosition(bodies_, dt);

  for (int i = 0; i < config_.positionIterations; i++) {
    solvePosition(activeManifolds, bodies_, config_);
  }
}
