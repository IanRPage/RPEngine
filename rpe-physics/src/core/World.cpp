#include <core/World.hpp>

#include <algorithm>
#include <broadphase/DynamicBVHBroadphase.hpp>
#include <collision/Manifold.hpp>
#include <collision/MassProperties.hpp>
#include <dynamics/Integrator.hpp>
#include <limits>
#include <unordered_map>
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

void World::removeBody(BodyHandle handle) noexcept {
  if (!bodies_.isLive(handle)) { return; }
  for (const Manifold& m : manifoldCache_.activeManifolds()) {
    if (m.bodyA == handle) { bodies_.wake(m.bodyB); }
    if (m.bodyB == handle) { bodies_.wake(m.bodyA); }
  }
  bodies_.removeBody(handle);
}

void World::wakeBodiesTouchingRemoved() noexcept {
  for (const Manifold& m : manifoldCache_.activeManifolds()) {
    bool liveA = bodies_.isLive(m.bodyA);
    bool liveB = bodies_.isLive(m.bodyB);
    if (liveA && !liveB) { bodies_.wake(m.bodyA); }
    if (liveB && !liveA) { bodies_.wake(m.bodyB); }
  }
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

void World::propagateWakeThroughContacts() noexcept {
  if (bodies_.sleepingCount() == 0) { return; }

  std::span<Manifold> manifolds = manifoldCache_.activeManifolds();
  bool anyMixed = false;
  for (const Manifold& m : manifolds) {
    bool sa = bodies_.isSleeping(m.bodyA);
    bool sb = bodies_.isSleeping(m.bodyB);
    if ((sa && isAwakeDynamic(m.bodyB)) || (sb && isAwakeDynamic(m.bodyA))) {
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

  std::vector<bool> rootAwake(ufParent_.size(), false);
  for (BodyHandle h : bodies_.liveHandles()) {
    if (isAwakeDynamic(h)) { rootAwake[findRoot(h.index)] = true; }
  }
  for (BodyHandle h : bodies_.liveHandles()) {
    if (bodies_.isSleeping(h) && rootAwake[findRoot(h.index)]) {
      bodies_.wake(h);
    }
  }
}

void World::updateSleep(std::span<const Manifold> awakeManifolds,
                        float dt) noexcept {
  if (!config_.enableSleeping) { return; }

  const float linThresholdSq =
      config_.linearSleepThreshold * config_.linearSleepThreshold;
  const float angThresholdSq =
      config_.angularSleepThreshold * config_.angularSleepThreshold;

  resetUnionFind();
  for (const Manifold& m : awakeManifolds) {
    if (!isAwakeDynamic(m.bodyA) || !isAwakeDynamic(m.bodyB)) { continue; }
    unite(m.bodyA.index, m.bodyB.index);
  }

  constexpr float kInf = std::numeric_limits<float>::infinity();
  std::vector<float> islandMinTimer(ufParent_.size(), kInf);
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
    float& islandMin = islandMinTimer[findRoot(h.index)];
    islandMin = std::min(islandMin, timer);
  }

  std::unordered_map<uint32_t, std::vector<BodyHandle>> islands;
  for (BodyHandle h : bodies_.liveHandles()) {
    if (!isAwakeDynamic(h)) { continue; }
    uint32_t root = findRoot(h.index);
    if (islandMinTimer[root] >= config_.timeToSleep) {
      islands[root].push_back(h);
    }
  }
  for (const auto& [root, members] : islands) {
    bodies_.putIslandToSleep(members);
  }
}
void World::step(float dt) noexcept {
  if (!config_.enableSleeping) {
    if (bodies_.sleepingCount() > 0) { bodies_.wakeAll(); }
  } else {
    wakeBodiesTouchingRemoved();
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

  std::vector<BodyHandle> sleepingBefore;
  if (bodies_.sleepingCount() > 0) {
    for (BodyHandle h : bodies_.liveHandles()) {
      if (bodies_.isSleeping(h)) { sleepingBefore.push_back(h); }
    }
  }

  manifoldCache_.beginFrame();
  if (!sleepingBefore.empty()) {
    manifoldCache_.retainIf([this](const Manifold& m) {
      if (!bodies_.isLive(m.bodyA) || !bodies_.isLive(m.bodyB)) {
        return false;
      }
      bool anySleeping =
          bodies_.isSleeping(m.bodyA) || bodies_.isSleeping(m.bodyB);
      return anySleeping && !isAwakeDynamic(m.bodyA) &&
             !isAwakeDynamic(m.bodyB);
    });
  }

  for (const auto& [a, b] : pairs) {
    if ((bodies_.isSleeping(a) || bodies_.isSleeping(b)) &&
        !isAwakeDynamic(a) && !isAwakeDynamic(b)) {
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

  if (!sleepingBefore.empty()) {
    propagateWakeThroughContacts();
    Vec3f deltaVelocity = gravity_ * dt;
    for (BodyHandle h : sleepingBefore) {
      if (bodies_.isLive(h) && !bodies_.isSleeping(h)) {
        bodies_.linearVelocity(h) += deltaVelocity;
      }
    }
  }

  std::span<Manifold> allManifolds = manifoldCache_.activeManifolds();

  std::vector<Manifold> awakeStorage;
  std::vector<size_t> awakeIndices;
  std::span<Manifold> activeManifolds = allManifolds;
  if (bodies_.sleepingCount() > 0) {
    awakeStorage.reserve(allManifolds.size());
    awakeIndices.reserve(allManifolds.size());
    for (size_t i = 0; i < allManifolds.size(); ++i) {
      const Manifold& m = allManifolds[i];
      if (bodies_.isSleeping(m.bodyA) || bodies_.isSleeping(m.bodyB)) {
        continue;
      }
      awakeStorage.push_back(m);
      awakeIndices.push_back(i);
    }
    activeManifolds = std::span<Manifold>(awakeStorage);
  }

  std::vector<float> restitutionBias = prepareRestitutionBias(
      activeManifolds, bodies_, config_.restitutionVelocityThreshold);

  InvInertiaWorldCache invInertiaCache(bodies_);
  warmStart(activeManifolds, bodies_, invInertiaCache);
  for (int i = 0; i < config_.velocityIterations; i++) {
    solveVelocity(activeManifolds, bodies_, restitutionBias, invInertiaCache);
  }

  integratePosition(bodies_, dt);

  for (int i = 0; i < config_.positionIterations; i++) {
    solvePosition(activeManifolds, bodies_, config_);
  }

  for (size_t k = 0; k < awakeIndices.size(); ++k) {
    allManifolds[awakeIndices[k]] = awakeStorage[k];
  }

  updateSleep(activeManifolds, dt);
}
