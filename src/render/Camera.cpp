#include <render/Camera.hpp>

#include <algorithm>
#include <cmath>

namespace render {

Mat4f OrthographicCamera::viewProjection() const noexcept {
  float left = center.x - halfHeight * aspect;
  float right = center.x + halfHeight * aspect;
  float bottom = center.y - halfHeight;
  float top = center.y + halfHeight;

  Vec3f eye{center.x, center.y, 1.0f};
  Vec3f target{center.x, center.y, 0.0f};
  Mat4f view = glm::lookAt(eye, target, Vec3f(0.0f, 1.0f, 0.0f));
  Mat4f projection = glm::ortho(left, right, bottom, top, nearPlane, farPlane);
  return projection * view;
}

Mat4f PerspectiveCamera::viewProjection() const noexcept {
  Mat4f view = glm::lookAt(position, target, up);
  Mat4f projection = glm::perspective(fovYRadians, aspect, nearPlane, farPlane);
  return projection * view;
}

Vec3f FreeFlyCameraController::forward() const noexcept {
  return glm::normalize(Vec3f(std::cos(pitch_) * std::sin(yaw_),
                              std::sin(pitch_),
                              -std::cos(pitch_) * std::cos(yaw_)));
}

Vec3f FreeFlyCameraController::right() const noexcept {
  return glm::normalize(glm::cross(forward(), Vec3f(0.0f, 1.0f, 0.0f)));
}

void FreeFlyCameraController::onMouseLook(Vec2f delta) noexcept {
  yaw_ += delta.x * kMouseSensitivity;
  pitch_ =
      std::clamp(pitch_ - delta.y * kMouseSensitivity, kMinPitch, kMaxPitch);
}

void FreeFlyCameraController::moveForward(float amount) noexcept {
  position_ += forward() * amount;
}

void FreeFlyCameraController::moveRight(float amount) noexcept {
  position_ += right() * amount;
}

void FreeFlyCameraController::moveUp(float amount) noexcept {
  position_ += Vec3f(0.0f, 1.0f, 0.0f) * amount;
}

PerspectiveCamera FreeFlyCameraController::toCamera(
    float aspect, float fovYRadians, float nearPlane,
    float farPlane) const noexcept {
  PerspectiveCamera camera;
  camera.position = position_;
  camera.target = position_ + forward();
  camera.up = Vec3f(0.0f, 1.0f, 0.0f);
  camera.aspect = aspect;
  camera.fovYRadians = fovYRadians;
  camera.nearPlane = nearPlane;
  camera.farPlane = farPlane;
  return camera;
}

}  // namespace render
