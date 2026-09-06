#ifndef RPENGINE_RENDER_INSTANCEDBATCH_HPP
#define RPENGINE_RENDER_INSTANCEDBATCH_HPP

#include <glad/gl.h>
#include <math/Types.hpp>
#include <render/Mesh.hpp>
#include <render/ShaderProgram.hpp>

#ifdef GLM_FORCE_QUAT_DATA_WXYZ
#error \
    "InstanceData relies on GLM's default x,y,z,w quat memory layout for raw GPU upload"
#endif

#include <vector>

namespace render {

struct InstanceData {
  Vec3f position;
  Quatf orientation;  // laid out (x, y, z, w) in memory
  Vec3f scale;
  Vec4f color;
};

class InstancedBatch {
 public:
  InstancedBatch();
  ~InstancedBatch();

  InstancedBatch(const InstancedBatch&) = delete;
  InstancedBatch& operator=(const InstancedBatch&) = delete;

  void begin() noexcept;
  void add(const InstanceData& instance);
  void render(const Mesh& mesh, const ShaderProgram& shader,
              const Mat4f& viewProjection);

  size_t size() const noexcept { return staging_.size(); }

 private:
  std::vector<InstanceData> staging_;
  GLuint instanceVBO_[2] = {0, 0};
  size_t bufferCapacity_[2] = {0, 0};  // curr bytes alloc'd / buffer
  int currentBuffer_ = 0;
};

}  // namespace render

#endif
