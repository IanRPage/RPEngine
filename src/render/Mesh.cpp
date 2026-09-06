#include <render/Mesh.hpp>

namespace render {

Mesh::Mesh(std::span<const Vertex> vertices, std::span<const uint32_t> indices)
    : indexCount_(static_cast<GLsizei>(indices.size())) {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
               vertices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
               indices.data(), GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, position)));

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, normal)));

  glBindVertexArray(0);
}

Mesh::~Mesh() { release(); }

Mesh::Mesh(Mesh&& other) noexcept
    : vao_(other.vao_),
      vbo_(other.vbo_),
      ebo_(other.ebo_),
      indexCount_(other.indexCount_) {
  other.vao_ = 0;
  other.vbo_ = 0;
  other.ebo_ = 0;
  other.indexCount_ = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
  if (this != &other) {
    release();
    vao_ = other.vao_;
    vbo_ = other.vbo_;
    ebo_ = other.ebo_;
    indexCount_ = other.indexCount_;
    other.vao_ = 0;
    other.vbo_ = 0;
    other.ebo_ = 0;
    other.indexCount_ = 0;
  }
  return *this;
}

void Mesh::release() noexcept {
  if (vao_ != 0) { glDeleteVertexArrays(1, &vao_); }
  if (vbo_ != 0) { glDeleteBuffers(1, &vbo_); }
  if (ebo_ != 0) { glDeleteBuffers(1, &ebo_); }
  vao_ = vbo_ = ebo_ = 0;
}

void Mesh::bind() const noexcept { glBindVertexArray(vao_); }

}  // namespace render
