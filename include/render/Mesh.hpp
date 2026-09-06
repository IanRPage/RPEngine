#ifndef RPENGINE_RENDER_MESH_HPP
#define RPENGINE_RENDER_MESH_HPP

#include <glad/gl.h>
#include <math/Types.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace render {

struct Vertex {
  Vec3f position;
  Vec3f normal;
};

struct MeshData {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
};

class Mesh {
 public:
  Mesh(std::span<const Vertex> vertices, std::span<const uint32_t> indices);
  explicit Mesh(const MeshData& data) : Mesh(data.vertices, data.indices) {}
  ~Mesh();

  Mesh(const Mesh&) = delete;
  Mesh& operator=(const Mesh&) = delete;
  Mesh(Mesh&& other) noexcept;
  Mesh& operator=(Mesh&& other) noexcept;

  void bind() const noexcept;
  GLsizei indexCount() const noexcept { return indexCount_; }
  GLuint vao() const noexcept { return vao_; }

 private:
  void release() noexcept;

  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
  GLsizei indexCount_ = 0;
};

}  // namespace render

#endif
