#ifndef RPENGINE_RENDER_SHADERPROGRAM_HPP
#define RPENGINE_RENDER_SHADERPROGRAM_HPP

#include <glad/gl.h>
#include <math/Types.hpp>

#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>

namespace render {

class ShaderProgram {
 public:
  ShaderProgram(std::string_view vertexSrc, std::string_view fragmentSrc);
  ~ShaderProgram();

  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator=(const ShaderProgram&) = delete;
  ShaderProgram(ShaderProgram&& other) noexcept;
  ShaderProgram& operator=(ShaderProgram&& other) noexcept;

  static ShaderProgram fromFiles(const std::filesystem::path& vertexPath,
                                 const std::filesystem::path& fragmentPath);

  void bind() const noexcept;

  void setUniform(std::string_view name, const Mat4f& value) const;
  void setUniform(std::string_view name, Vec4f value) const;
  void setUniform(std::string_view name, Vec3f value) const;
  void setUniform(std::string_view name, float value) const;

  GLuint id() const noexcept { return program_; }

 private:
  GLint uniformLocation(std::string_view name) const;

  GLuint program_ = 0;
  mutable std::unordered_map<std::string, GLint> uniformCache_;
};

}  // namespace render

#endif
