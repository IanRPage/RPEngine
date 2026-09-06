#ifndef RPENGINE_RENDER_CAMERA_HPP
#define RPENGINE_RENDER_CAMERA_HPP

#include <math/Types.hpp>

namespace render {

struct OrthographicCamera {
  Vec2f center{0.0f, 0.0f};
  float halfHeight = 10.0f;
  float aspect = 16.0f / 9.0f;
  float nearPlane = 0.1f;
  float farPlane = 100.0f;

  Mat4f viewProjection() const noexcept;
};

struct PerspectiveCamera {
  Vec3f position{0.0f, 0.0f, 10.0f};
  Vec3f target{0.0f, 0.0f, 0.0f};
  Vec3f up{0.0f, 1.0f, 0.0f};
  float fovYRadians = glm::radians(60.0f);
  float aspect = 16.0f / 9.0f;
  float nearPlane = 0.1f;
  float farPlane = 500.0f;

  Mat4f viewProjection() const noexcept;
};

class FreeFlyCameraController {
 public:
  void onMouseLook(Vec2f delta) noexcept;
  void moveForward(float amount) noexcept;
  void moveRight(float amount) noexcept;
  void moveUp(float amount) noexcept;

  PerspectiveCamera toCamera(float aspect,
                             float fovYRadians = glm::radians(60.0f),
                             float nearPlane = 0.1f,
                             float farPlane = 500.0f) const noexcept;

  Vec3f position() const noexcept { return position_; }
  void setPosition(Vec3f position) noexcept { position_ = position; }

 private:
  Vec3f forward() const noexcept;
  Vec3f right() const noexcept;

  Vec3f position_{0.0f, 4.0f, 18.0f};
  float yaw_ = 0.0f;  // 0 faces -Z
  float pitch_ = 0.0f;

  static constexpr float kMinPitch = -1.5f;
  static constexpr float kMaxPitch = 1.5f;
  static constexpr float kMouseSensitivity = 0.005f;
};

}  // namespace render

#endif
