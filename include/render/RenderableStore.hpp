#ifndef RPENGINE_RENDER_RENDERABLESTORE_HPP
#define RPENGINE_RENDER_RENDERABLESTORE_HPP

#include <core/BodyHandle.hpp>
#include <math/Types.hpp>

#include <cstdint>
#include <vector>

namespace render {

class RenderableStore {
 public:
  static constexpr Vec4f kDefaultColor{0.8f, 0.8f, 0.8f, 1.0f};

  void setColor(BodyHandle handle, Vec4f color);
  Vec4f colorOr(BodyHandle handle,
                Vec4f fallback = kDefaultColor) const noexcept;
  bool hasColor(BodyHandle handle) const noexcept;
  void clear() noexcept;

 private:
  struct Entry {
    Vec4f color = kDefaultColor;
    uint32_t generation = 0;
    bool assigned = false;
  };

  std::vector<Entry> entries_;
};

}  // namespace render

#endif
