#include <core/World.hpp>

#include <algorithm>
#include <cmath>
#include <broadphase/DynamicBVHBroadphase.hpp>
#include <collision/Manifold.hpp>
#include <collision/MassProperties.hpp>
#include <dynamics/Integrator.hpp>
#include <limits>
#include <vector>

namespace {
float boundingRadiusOf(const ShapeVariant& shape) noexcept {
  return std::visit([](const auto& s) { return s.boundingRadius(); }, shape);
}

bool aabbOverlap(const AABB& x, const AABB& y) noexcept {
  return x.min.x <= y.max.x && x.max.x >= y.min.x && x.min.y <= y.max.y &&
         x.max.y >= y.min.y && x.min.z <= y.max.z && x.max.z >= y.min.z;
}

std::size_t childCount(const ShapeVariant& shape) noexcept {
  const auto* c = std::get_if<CompoundShape>(&shape);
  return c ? c->children.size() : 1;
}

const ShapeVariant& childShape(const ShapeVariant& shape,
                               std::size_t i) noexcept {
  const auto* c = std::get_if<CompoundShape>(&shape);
  return c ? c->children[i].shape : shape;
}

const Transform* childLocal(const ShapeVariant& shape,
                            std::size_t i) noexcept {
  const auto* c = std::get_if<CompoundShape>(&shape);
  return c ? &c->children[i].localTransform : nullptr;
}

AABB paddedAABB(const AABB& aabb) noexcept {
  Vec3f extent = aabb.max - aabb.min;
  float scale = std::max({1.0f, extent.x, extent.y, extent.z});
  Vec3f pad(1e-4f * scale);
  return AABB(aabb.min - pad, aabb.max + pad);
}

bool normalizeRay(const Ray& ray, float maxDistance, Ray& out) noexcept {
  if (!(maxDistance >= 0.0f)) { return false; }
  float len = glm::length(ray.direction);
  if (!(len > 0.0f) || !std::isfinite(len)) { return false; }
  out = Ray{ray.origin, ray.direction / len};
  return true;
}

std::unique_ptr<IBroadphase> normalizeBroadphase(
    std::unique_ptr<IBroadphase> broadphase) noexcept {
  return broadphase ? std::move(broadphase)
                    : std::make_unique<DynamicBVHBroadphase>();
}

bool hasMovedSincePrev(const BodyStore& bodies, BodyHandle h) noexcept {
  return bodies.position(h) != bodies.prevPosition(h) ||
         bodies.orientation(h) != bodies.prevOrientation(h);
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
    if (bodies_.invMass(h) <= 0.0f || bodies_.isSleeping(h)) { continue; }
    bodies_.prevPosition(h) = bodies_.position(h);
    bodies_.prevOrientation(h) = bodies_.orientation(h);
  }
}

void World::collideLeafPair(BodyHandle a, const ShapeVariant& leafA,
                            const Transform& ta, uint32_t idxA,
                            const Transform* localA, BodyHandle b,
                            const ShapeVariant& leafB, const Transform& tb,
                            uint32_t idxB, const Transform* localB) noexcept {
  GjkResult gjk = gjkFn_(leafA, ta, leafB, tb);
  if (!gjk.overlapping) { return; }

  EpaResult epa = epaFn_(leafA, ta, leafB, tb, gjk);
  Manifold fresh = buildManifold(leafA, ta, leafB, tb, a, b, gjk, epa);
  fresh.childIndexA = idxA;
  fresh.childIndexB = idxB;
  if (localA || localB) {
    for (uint8_t k = 0; k < fresh.pointCount; ++k) {
      ManifoldPoint& p = fresh.points[k];
      if (localA) { p.localAnchorA = transformPoint(*localA, p.localAnchorA); }
      if (localB) { p.localAnchorB = transformPoint(*localB, p.localAnchorB); }
    }
  }

  float matchThreshold =
      kManifoldMatchFactor *
      std::min(boundingRadiusOf(leafA), boundingRadiusOf(leafB));
  manifoldCache_.updateManifold(fresh, matchThreshold);
}

void World::collideCompoundPair(BodyHandle a, const ShapeVariant& shapeA,
                                const Transform& ta, BodyHandle b,
                                const ShapeVariant& shapeB,
                                const Transform& tb) noexcept {
  const std::size_t countA = childCount(shapeA);
  const std::size_t countB = childCount(shapeB);

  childScratchB_.clear();
  childScratchB_.reserve(countB);
  for (std::size_t j = 0; j < countB; ++j) {
    const Transform* localB = childLocal(shapeB, j);
    Transform tj = localB ? compose(tb, *localB) : tb;
    childScratchB_.push_back(ChildProxy{tj, worldAABB(childShape(shapeB, j), tj)});
  }

  for (std::size_t i = 0; i < countA; ++i) {
    const ShapeVariant& leafA = childShape(shapeA, i);
    const Transform* localA = childLocal(shapeA, i);
    Transform ti = localA ? compose(ta, *localA) : ta;
    AABB boxI = worldAABB(leafA, ti);

    for (std::size_t j = 0; j < countB; ++j) {
      const ChildProxy& proxyB = childScratchB_[j];
      if (!aabbOverlap(boxI, proxyB.aabb)) { continue; }
      collideLeafPair(a, leafA, ti, static_cast<uint32_t>(i), localA, b,
                      childShape(shapeB, j), proxyB.transform,
                      static_cast<uint32_t>(j), childLocal(shapeB, j));
    }
  }
}

bool World::isAwakeDynamic(BodyHandle h) const noexcept {
  return bodies_.isLive(h) && bodies_.invMass(h) > 0.0f &&
         !bodies_.isSleeping(h);
}

bool World::wakesContactPartner(BodyHandle h) const noexcept {
  if (!bodies_.isLive(h)) { return true; }
  return bodies_.invMass(h) > 0.0f ? !bodies_.isSleeping(h)
                                   : hasMovedSincePrev(bodies_, h);
}

void World::resetUnionFind() noexcept {
  uint32_t maxIndex = 0;
  for (BodyHandle h : bodies_.liveHandles()) {
    maxIndex = std::max(maxIndex, h.index + 1);
  }
  ufParent_.resize(maxIndex);
  for (uint32_t i = 0; i < maxIndex; ++i) { ufParent_[i] = i; }
}

uint32_t World::findRoot(uint32_t i) noexcept {
  while (ufParent_[i] != i) {
    ufParent_[i] = ufParent_[ufParent_[i]];
    i = ufParent_[i];
  }
  return i;
}

void World::unite(uint32_t a, uint32_t b) noexcept {
  a = findRoot(a);
  b = findRoot(b);
  if (a != b) { ufParent_[std::max(a, b)] = std::min(a, b); }
}

void World::propagateWakeThroughContacts(Vec3f wakeDeltaVelocity) noexcept {
  if (bodies_.sleepingCount() == 0) { return; }

  std::span<Manifold> manifolds = manifoldCache_.activeManifolds();
  bool anyMixed = false;
  for (const Manifold& m : manifolds) {
    if ((bodies_.isSleeping(m.bodyA) && wakesContactPartner(m.bodyB)) ||
        (bodies_.isSleeping(m.bodyB) && wakesContactPartner(m.bodyA))) {
      anyMixed = true;
      break;
    }
  }
  if (!anyMixed) { return; }

  resetUnionFind();
  for (const Manifold& m : manifolds) {
    if (!bodies_.isLive(m.bodyA) || !bodies_.isLive(m.bodyB)) { continue; }
    if (bodies_.invMass(m.bodyA) <= 0.0f || bodies_.invMass(m.bodyB) <= 0.0f) {
      continue;
    }
    unite(m.bodyA.index, m.bodyB.index);
  }

  ufAwake_.assign(ufParent_.size(), 0);
  for (const Manifold& m : manifolds) {
    if (bodies_.isSleeping(m.bodyA) && wakesContactPartner(m.bodyB)) {
      ufAwake_[findRoot(m.bodyA.index)] = 1;
    }
    if (bodies_.isSleeping(m.bodyB) && wakesContactPartner(m.bodyA)) {
      ufAwake_[findRoot(m.bodyB.index)] = 1;
    }
  }
  for (BodyHandle h : bodies_.liveHandles()) {
    if (bodies_.isSleeping(h) && ufAwake_[findRoot(h.index)]) {
      bodies_.wake(h);
      bodies_.linearVelocity(h) += wakeDeltaVelocity;
    }
  }
}

void World::updateSleep(std::span<const Manifold> manifolds,
                        float dt) noexcept {
  if (!config_.enableSleeping) { return; }

  const float linThresholdSq =
      config_.linearSleepThreshold * config_.linearSleepThreshold;
  const float angThresholdSq =
      config_.angularSleepThreshold * config_.angularSleepThreshold;

  resetUnionFind();
  for (const Manifold& m : manifolds) {
    if (!isAwakeDynamic(m.bodyA) || !isAwakeDynamic(m.bodyB)) { continue; }
    unite(m.bodyA.index, m.bodyB.index);
  }

  constexpr float kInf = std::numeric_limits<float>::infinity();
  ufMinTimer_.assign(ufParent_.size(), kInf);
  for (BodyHandle h : bodies_.liveHandles()) {
    if (!isAwakeDynamic(h)) { continue; }
    float linSq =
        glm::dot(bodies_.linearVelocity(h), bodies_.linearVelocity(h));
    float angSq =
        glm::dot(bodies_.angularVelocity(h), bodies_.angularVelocity(h));
    float& timer = bodies_.sleepTimer(h);
    if (linSq > linThresholdSq || angSq > angThresholdSq) {
      timer = 0.0f;
    } else {
      timer += dt;
    }
    float& islandMin = ufMinTimer_[findRoot(h.index)];
    islandMin = std::min(islandMin, timer);
  }

  for (BodyHandle h : bodies_.liveHandles()) {
    if (isAwakeDynamic(h) &&
        ufMinTimer_[findRoot(h.index)] >= config_.timeToSleep) {
      bodies_.putToSleep(h);
    }
  }
}

void World::step(float dt) noexcept {
  if (!config_.enableSleeping) {
    if (bodies_.sleepingCount() > 0) { bodies_.wakeAll(); }
  } else {
    for (BodyHandle h : bodies_.liveHandles()) {
      if (bodies_.isSleeping(h) && hasMovedSincePrev(bodies_, h)) {
        bodies_.wake(h);
      }
    }
    propagateWakeThroughContacts();
  }

  integrateVelocity(bodies_, dt, gravity_);

  for (BodyHandle h : bodies_.liveHandles()) {
    if (bodies_.isSleeping(h)) { continue; }
    Transform t = bodies_.transform(h);
    AABB aabb = worldAABB(bodies_.shape(h), t);
    Vec3f displacement = bodies_.linearVelocity(h) * dt;
    bodies_.setAABB(h, aabb, displacement);
  }

  auto pairs = broadphase_->computePairs(bodies_);

  manifoldCache_.beginFrame();
  if (bodies_.sleepingCount() > 0) {
    manifoldCache_.retainIf([this](const Manifold& m) {
      return (bodies_.isSleeping(m.bodyA) || bodies_.isSleeping(m.bodyB)) &&
             !wakesContactPartner(m.bodyA) && !wakesContactPartner(m.bodyB);
    });
  }

  for (const auto& [a, b] : pairs) {
    if ((bodies_.isSleeping(a) || bodies_.isSleeping(b)) &&
        !wakesContactPartner(a) && !wakesContactPartner(b)) {
      continue;
    }

    const ShapeVariant& shapeA = bodies_.shape(a);
    const ShapeVariant& shapeB = bodies_.shape(b);
    Transform ta = bodies_.transform(a);
    Transform tb = bodies_.transform(b);

    if (!isCompound(shapeA) && !isCompound(shapeB)) {
      collideLeafPair(a, shapeA, ta, 0, nullptr, b, shapeB, tb, 0, nullptr);
    } else {
      collideCompoundPair(a, shapeA, ta, b, shapeB, tb);
    }
  }
  manifoldCache_.endFrame();

  propagateWakeThroughContacts(gravity_ * dt);
  for (BodyHandle h : bodies_.liveHandles()) {
    if (bodies_.invMass(h) <= 0.0f && hasMovedSincePrev(bodies_, h)) {
      bodies_.prevPosition(h) = bodies_.position(h);
      bodies_.prevOrientation(h) = bodies_.orientation(h);
    }
  }

  std::span<Manifold> activeManifolds = manifoldCache_.activeManifolds();

  std::vector<float> restitutionBias = prepareRestitutionBias(
      activeManifolds, bodies_, config_.restitutionVelocityThreshold);

  if (!restitutionBias.empty()) {
    InvInertiaWorldCache invInertiaCache(bodies_);
    warmStart(activeManifolds, bodies_, invInertiaCache);
    for (int i = 0; i < config_.velocityIterations; i++) {
      solveVelocity(activeManifolds, bodies_, restitutionBias, invInertiaCache);
    }
  }

  integratePosition(bodies_, dt);

  if (!restitutionBias.empty()) {
    for (int i = 0; i < config_.positionIterations; i++) {
      solvePosition(activeManifolds, bodies_, config_);
    }
  }

  updateSleep(activeManifolds, dt);
}

std::optional<WorldRayHit> World::raycastClosest(const Ray& ray,
                                                 float maxDistance) const {
  Ray unitRay;
  if (!normalizeRay(ray, maxDistance, unitRay)) { return std::nullopt; }

  std::optional<WorldRayHit> closest;
  float bestDistance = maxDistance;
  for (BodyHandle h : bodies_.liveHandles()) {
    const ShapeVariant& shape = bodies_.shape(h);
    Transform t = bodies_.transform(h);
    if (!rayIntersectsAABB(unitRay, paddedAABB(worldAABB(shape, t)),
                           bestDistance)) {
      continue;
    }
    RayHit hit = raycastShape(shape, t, unitRay, bestDistance);
    if (!hit.hit) { continue; }
    if (closest && hit.distance == closest->distance &&
        !(h < closest->body)) {
      continue;
    }
    closest = WorldRayHit{h, hit.distance, hit.point, hit.normal};
    bestDistance = hit.distance;
  }
  return closest;
}

std::vector<WorldRayHit> World::raycastAll(const Ray& ray,
                                           float maxDistance) const {
  std::vector<WorldRayHit> hits;
  Ray unitRay;
  if (!normalizeRay(ray, maxDistance, unitRay)) { return hits; }

  for (BodyHandle h : bodies_.liveHandles()) {
    const ShapeVariant& shape = bodies_.shape(h);
    Transform t = bodies_.transform(h);
    if (!rayIntersectsAABB(unitRay, paddedAABB(worldAABB(shape, t)),
                           maxDistance)) {
      continue;
    }
    RayHit hit = raycastShape(shape, t, unitRay, maxDistance);
    if (hit.hit) {
      hits.push_back(WorldRayHit{h, hit.distance, hit.point, hit.normal});
    }
  }

  std::sort(hits.begin(), hits.end(),
            [](const WorldRayHit& a, const WorldRayHit& b) {
              if (a.distance != b.distance) { return a.distance < b.distance; }
              return a.body < b.body;
            });
  return hits;
}

std::vector<BodyHandle> World::queryAABB(const AABB& aabb) const {
  std::vector<BodyHandle> result;
  for (BodyHandle h : bodies_.liveHandles()) {
    AABB bodyAABB = worldAABB(bodies_.shape(h), bodies_.transform(h));
    if (aabbOverlap(bodyAABB, aabb)) { result.push_back(h); }
  }
  std::sort(result.begin(), result.end());
  return result;
}
