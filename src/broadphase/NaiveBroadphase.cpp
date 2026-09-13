#include <broadphase/NaiveBroadphase.hpp>

#include <utility>

std::span<const std::pair<BodyHandle, BodyHandle>>
NaiveBroadphase::computePairs(const BodyStore& bodies) {
  pairs_.clear();

  const std::vector<BodyHandle>& handles = bodies.liveHandles();
  for (size_t i = 0; i < handles.size(); i++) {
    for (size_t j = i + 1; j < handles.size(); j++) {
      BodyHandle a = handles[i];
      BodyHandle b = handles[j];
      if (b < a) { std::swap(a, b); }
      pairs_.emplace_back(a, b);
    }
  }

  return pairs_;
}
