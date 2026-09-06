#include <collision/ManifoldCache.hpp>

#include <algorithm>
#include <cstdint>

size_t ManifoldCache::PairHash::operator()(
    const std::pair<BodyHandle, BodyHandle>& p) const noexcept {
  BodyHandleHash h;
  size_t ha = h(p.first);
  size_t hb = h(p.second);
  return ha ^ (hb + 0x9e3779b9 + (ha << 6) + (ha >> 2));
}

std::pair<BodyHandle, BodyHandle> ManifoldCache::canonicalize(
    BodyHandle a, BodyHandle b) noexcept {
  return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

Manifold& ManifoldCache::getOrCreate(BodyHandle a, BodyHandle b) noexcept {
  auto key = canonicalize(a, b);
  auto it = indexOf_.find(key);
  std::size_t idx;
  if (it != indexOf_.end()) {
    idx = it->second;
  } else {
    idx = manifolds_.size();
    Manifold m{};
    m.bodyA = key.first;
    m.bodyB = key.second;
    manifolds_.push_back(m);
    visited_.push_back(false);
    indexOf_.emplace(key, idx);
  }
  visited_[idx] = true;
  return manifolds_[idx];
}

void ManifoldCache::updateManifold(const Manifold& freshManifold,
                                   const Transform& transformA,
                                   float matchThreshold) noexcept {
  Manifold& cached = getOrCreate(freshManifold.bodyA, freshManifold.bodyB);
  Manifold oldSnapshot = cached;

  Manifold merged = freshManifold;
  for (std::uint8_t i = 0; i < merged.pointCount; ++i) {
    Vec3f newAnchorWorld = transformPoint(transformA, merged.points[i].localAnchorA);
    float bestDist = matchThreshold;
    int bestJ = -1;
    for (std::uint8_t j = 0; j < oldSnapshot.pointCount; ++j) {
      Vec3f oldAnchorWorld =
          transformPoint(transformA, oldSnapshot.points[j].localAnchorA);
      float d = glm::length(oldAnchorWorld - newAnchorWorld);
      if (d < bestDist) {
        bestDist = d;
        bestJ = static_cast<int>(j);
      }
    }
    if (bestJ >= 0) {
      merged.points[i].normalImpulse = oldSnapshot.points[bestJ].normalImpulse;
      merged.points[i].tangentImpulse[0] =
          oldSnapshot.points[bestJ].tangentImpulse[0];
      merged.points[i].tangentImpulse[1] =
          oldSnapshot.points[bestJ].tangentImpulse[1];
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
    auto removedKey = canonicalize(manifolds_[i].bodyA, manifolds_[i].bodyB);
    indexOf_.erase(removedKey);
    std::size_t lastIdx = manifolds_.size() - 1;
    if (i != lastIdx) {
      manifolds_[i] = manifolds_[lastIdx];
      visited_[i] = visited_[lastIdx];
      auto movedKey = canonicalize(manifolds_[i].bodyA, manifolds_[i].bodyB);
      indexOf_[movedKey] = i;
    }
    manifolds_.pop_back();
    visited_.pop_back();
  }
}

std::span<Manifold> ManifoldCache::activeManifolds() noexcept {
  return std::span<Manifold>(manifolds_.data(), manifolds_.size());
}
