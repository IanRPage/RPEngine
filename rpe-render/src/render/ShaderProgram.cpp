#include <render/ShaderProgram.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace render {

namespace {

GLuint compileStage(GLenum stage, std::string_view source,
                     const char* stageName) {
  GLuint shader = glCreateShader(stage);
  const char* srcPtr = source.data();
  GLint srcLen = static_cast<GLint>(source.size());
  glShaderSource(shader, 1, &srcPtr, &srcLen);
  glCompileShader(shader);

  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_FALSE) {
    GLint logLen = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLen);
    std::string log(static_cast<size_t>(logLen), '\0');
    glGetShaderInfoLog(shader, logLen, nullptr, log.data());
    glDeleteShader(shader);
    throw std::runtime_error(std::string("ShaderProgram: ") + stageName +
                              " shader compile failed:\n" + log);
  }
  return shader;
}

std::string readFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("ShaderProgram: could not open shader file: " +
                              path.string());
  }
  std::ostringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

}  // namespace

ShaderProgram::ShaderProgram(std::string_view vertexSrc,
                              std::string_view fragmentSrc) {
  GLuint vertexShader = compileStage(GL_VERTEX_SHADER, vertexSrc, "vertex");
  GLuint fragmentShader;
  try {
    fragmentShader = compileStage(GL_FRAGMENT_SHADER, fragmentSrc, "fragment");
  } catch (...) {
    glDeleteShader(vertexShader);
    throw;
  }

  program_ = glCreateProgram();
  glAttachShader(program_, vertexShader);
  glAttachShader(program_, fragmentShader);
  glLinkProgram(program_);

  GLint ok = GL_FALSE;
  glGetProgramiv(program_, GL_LINK_STATUS, &ok);

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  if (ok == GL_FALSE) {
    GLint logLen = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &logLen);
    std::string log(static_cast<size_t>(logLen), '\0');
    glGetProgramInfoLog(program_, logLen, nullptr, log.data());
    glDeleteProgram(program_);
    program_ = 0;
    throw std::runtime_error("ShaderProgram: program link failed:\n" + log);
  }
}

ShaderProgram::~ShaderProgram() {
  if (program_ != 0) {
    glDeleteProgram(program_);
  }
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
    : program_(other.program_), uniformCache_(std::move(other.uniformCache_)) {
  other.program_ = 0;
}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
  if (this != &other) {
    if (program_ != 0) {
      glDeleteProgram(program_);
    }
    program_ = other.program_;
    uniformCache_ = std::move(other.uniformCache_);
    other.program_ = 0;
  }
  return *this;
}

ShaderProgram ShaderProgram::fromFiles(
    const std::filesystem::path& vertexPath,
    const std::filesystem::path& fragmentPath) {
  std::string vertexSrc = readFile(vertexPath);
  std::string fragmentSrc = readFile(fragmentPath);
  return ShaderProgram(vertexSrc, fragmentSrc);
}

void ShaderProgram::bind() const noexcept { glUseProgram(program_); }

GLint ShaderProgram::uniformLocation(std::string_view name) const {
  std::string key(name);
  auto it = uniformCache_.find(key);
  if (it != uniformCache_.end()) {
    return it->second;
  }
  GLint location = glGetUniformLocation(program_, key.c_str());
  uniformCache_.emplace(std::move(key), location);
  return location;
}

void ShaderProgram::setUniform(std::string_view name,
                                const Mat4f& value) const {
  glUniformMatrix4fv(uniformLocation(name), 1, GL_FALSE, &value[0][0]);
}

void ShaderProgram::setUniform(std::string_view name, Vec4f value) const {
  glUniform4fv(uniformLocation(name), 1, &value[0]);
}

void ShaderProgram::setUniform(std::string_view name, Vec3f value) const {
  glUniform3fv(uniformLocation(name), 1, &value[0]);
}

void ShaderProgram::setUniform(std::string_view name, float value) const {
  glUniform1f(uniformLocation(name), value);
}

}  // namespace render
