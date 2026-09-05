#include <broadphase/NaiveBroadphase.hpp>

std::span<const std::pair<BodyHandle, BodyHandle>>
NaiveBroadphase::computePairs(const BodyStore& bodies) {
  pairs_.clear();

  const std::vector<BodyHandle>& handles = bodies.liveHandles();
  for (size_t i = 0; i < handles.size(); i++) {
    for (size_t j = i + 1; j < handles.size(); j++) {
      pairs_.emplace_back(handles[i], handles[j]);
    }
  }

  return pairs_;
}
