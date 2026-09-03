#ifndef RPENGINE_CORE_BODYHANDLE_HPP
#define RPENGINE_CORE_BODYHANDLE_HPP

#include <cstdint>
#include <functional>
#include <limits>

struct BodyHandle {
  static constexpr uint32_t kInvalid = std::numeric_limits<uint32_t>::max();

  uint32_t index = kInvalid;
  uint32_t generation = 0;

  bool operator==(const BodyHandle&) const noexcept = default;
  bool valid() const noexcept { return index != kInvalid; }

  bool operator<(const BodyHandle& other) const noexcept {
    return index < other.index;
  }
};

struct BodyHandleHash {
  size_t operator()(const BodyHandle& h) const noexcept {
    return std::hash<uint64_t>{}((static_cast<uint64_t>(h.index) << 32) |
                                 h.generation);
  }
};

#endif
