#include <collision/ManifoldCache.hpp>

#include <algorithm>
#include <cstdint>

size_t ManifoldCache::KeyHash::operator()(const Key& k) const noexcept {
  BodyHandleHash h;
  size_t seed = h(k.first);
  auto mix = [&seed](size_t v) {
    seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  };
  mix(h(k.second));
  mix(std::hash<uint64_t>{}((static_cast<uint64_t>(k.childFirst) << 32) |
                            k.childSecond));
  return seed;
}

ManifoldCache::Key ManifoldCache::canonicalize(BodyHandle a, BodyHandle b,
                                               uint32_t childA,
                                               uint32_t childB) noexcept {
  if (a < b) { return Key{a, b, childA, childB}; }
  return Key{b, a, childB, childA};
}

ManifoldCache::Key ManifoldCache::keyOf(const Manifold& m) noexcept {
  return canonicalize(m.bodyA, m.bodyB, m.childIndexA, m.childIndexB);
}

Manifold& ManifoldCache::getOrCreate(BodyHandle a, BodyHandle b) noexcept {
  return getOrCreate(a, b, 0, 0);
}

Manifold& ManifoldCache::getOrCreate(BodyHandle a, BodyHandle b,
                                     uint32_t childA,
                                     uint32_t childB) noexcept {
  Key key = canonicalize(a, b, childA, childB);
  auto it = indexOf_.find(key);
  std::size_t idx;
  if (it != indexOf_.end()) {
    idx = it->second;
  } else {
    idx = manifolds_.size();
    Manifold m{};
    m.bodyA = key.first;
    m.bodyB = key.second;
    m.childIndexA = key.childFirst;
    m.childIndexB = key.childSecond;
    manifolds_.push_back(m);
    visited_.push_back(false);
    indexOf_.emplace(key, idx);
  }
  visited_[idx] = true;
  return manifolds_[idx];
}

void ManifoldCache::updateManifold(const Manifold& freshManifold,
                                   float matchThreshold) noexcept {
  Manifold& cached =
      getOrCreate(freshManifold.bodyA, freshManifold.bodyB,
                  freshManifold.childIndexA, freshManifold.childIndexB);
  Manifold oldSnapshot = cached;

  const bool swapped = oldSnapshot.pointCount > 0 &&
                       !(oldSnapshot.bodyA == freshManifold.bodyA);

  Manifold merged = freshManifold;
  for (std::uint8_t i = 0; i < merged.pointCount; ++i) {
    const Vec3f& newAnchor = merged.points[i].localAnchorA;
    float bestDist = matchThreshold;
    int bestJ = -1;
    for (std::uint8_t j = 0; j < oldSnapshot.pointCount; ++j) {
      const Vec3f& oldAnchor = swapped ? oldSnapshot.points[j].localAnchorB
                                       : oldSnapshot.points[j].localAnchorA;
      float d = glm::length(oldAnchor - newAnchor);
      if (d < bestDist) {
        bestDist = d;
        bestJ = static_cast<int>(j);
      }
    }
    if (bestJ >= 0) {
      merged.points[i].normalImpulse = oldSnapshot.points[bestJ].normalImpulse;
      if (!swapped) {
        merged.points[i].tangentImpulse[0] =
            oldSnapshot.points[bestJ].tangentImpulse[0];
        merged.points[i].tangentImpulse[1] =
            oldSnapshot.points[bestJ].tangentImpulse[1];
      }
    }
  }
  cached = merged;
}

void ManifoldCache::beginFrame() noexcept {
  std::fill(visited_.begin(), visited_.end(), false);
}

void ManifoldCache::endFrame() noexcept {
  std::size_t i = 0;
  while (i < manifolds_.size()) {
    if (visited_[i]) {
      ++i;
      continue;
    }
    indexOf_.erase(keyOf(manifolds_[i]));
    std::size_t lastIdx = manifolds_.size() - 1;
    if (i != lastIdx) {
      manifolds_[i] = manifolds_[lastIdx];
      visited_[i] = visited_[lastIdx];
      indexOf_[keyOf(manifolds_[i])] = i;
    }
    manifolds_.pop_back();
    visited_.pop_back();
  }
}

std::span<Manifold> ManifoldCache::activeManifolds() noexcept {
  return std::span<Manifold>(manifolds_.data(), manifolds_.size());
}
