#include <render/RenderableStore.hpp>

namespace render {

void RenderableStore::setColor(BodyHandle handle, Vec4f color) {
  if (entries_.size() <= handle.index) {
    entries_.resize(static_cast<size_t>(handle.index) + 1);
  }
  entries_[handle.index] = Entry{color, handle.generation, true};
}

Vec4f RenderableStore::colorOr(BodyHandle handle, Vec4f fallback) const noexcept {
  if (handle.index >= entries_.size()) { return fallback; }
  const Entry& entry = entries_[handle.index];
  if (!entry.assigned || entry.generation != handle.generation) {
    return fallback;
  }
  return entry.color;
}

bool RenderableStore::hasColor(BodyHandle handle) const noexcept {
  if (handle.index >= entries_.size()) { return false; }
  const Entry& entry = entries_[handle.index];
  return entry.assigned && entry.generation == handle.generation;
}

void RenderableStore::clear() noexcept { entries_.clear(); }

}  // namespace render
