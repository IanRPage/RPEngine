#include <render/InstancedBatch.hpp>

#include <cstddef>

namespace render {

namespace {
constexpr float kGrowthFactor = 1.5f;
}

InstancedBatch::InstancedBatch() { glGenBuffers(2, instanceVBO_); }

InstancedBatch::~InstancedBatch() { glDeleteBuffers(2, instanceVBO_); }

void InstancedBatch::begin() noexcept { staging_.clear(); }

void InstancedBatch::add(const InstanceData& instance) {
  staging_.push_back(instance);
}

void InstancedBatch::render(const Mesh& mesh, const ShaderProgram& shader,
                            const Mat4f& viewProjection) {
  if (staging_.empty()) { return; }

  shader.bind();
  shader.setUniform("uViewProjection", viewProjection);

  mesh.bind();

  GLuint vbo = instanceVBO_[currentBuffer_];
  size_t requiredBytes = staging_.size() * sizeof(InstanceData);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);

  if (requiredBytes > bufferCapacity_[currentBuffer_]) {
    bufferCapacity_[currentBuffer_] =
        static_cast<size_t>(static_cast<float>(requiredBytes) * kGrowthFactor);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(bufferCapacity_[currentBuffer_]),
                 nullptr, GL_STREAM_DRAW);
  } else {
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(bufferCapacity_[currentBuffer_]),
                 nullptr, GL_STREAM_DRAW);
  }
  glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(requiredBytes),
                  staging_.data());

  constexpr GLsizei stride = sizeof(InstanceData);
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(
      2, 3, GL_FLOAT, GL_FALSE, stride,
      reinterpret_cast<void*>(offsetof(InstanceData, position)));
  glVertexAttribDivisor(2, 1);

  glEnableVertexAttribArray(3);
  glVertexAttribPointer(
      3, 4, GL_FLOAT, GL_FALSE, stride,
      reinterpret_cast<void*>(offsetof(InstanceData, orientation)));
  glVertexAttribDivisor(3, 1);

  glEnableVertexAttribArray(4);
  glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void*>(offsetof(InstanceData, scale)));
  glVertexAttribDivisor(4, 1);

  glEnableVertexAttribArray(5);
  glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void*>(offsetof(InstanceData, color)));
  glVertexAttribDivisor(5, 1);

  glDrawElementsInstanced(GL_TRIANGLES, mesh.indexCount(), GL_UNSIGNED_INT,
                          nullptr, static_cast<GLsizei>(staging_.size()));

  currentBuffer_ = 1 - currentBuffer_;
}

}  // namespace render
