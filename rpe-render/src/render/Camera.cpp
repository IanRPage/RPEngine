#include <render/Camera.hpp>

#include <algorithm>
#include <cmath>

namespace render {

namespace {

constexpr float kOrthoEyeZ = 1.0f;

Vec2f viewportToNdc(Vec2f viewportPos, Vec2f viewportSize) noexcept {
  float w = std::max(viewportSize.x, 1.0f);
  float h = std::max(viewportSize.y, 1.0f);
  return Vec2f(2.0f * viewportPos.x / w - 1.0f, 1.0f - 2.0f * viewportPos.y / h);
}

}  // namespace

Mat4f OrthographicCamera::viewProjection() const noexcept {
  float left = -halfHeight * aspect;
  float right = halfHeight * aspect;
  float bottom = -halfHeight;
  float top = halfHeight;

  Vec3f eye{center.x, center.y, kOrthoEyeZ};
  Vec3f target{center.x, center.y, 0.0f};
  Mat4f view = glm::lookAt(eye, target, Vec3f(0.0f, 1.0f, 0.0f));
  Mat4f projection = glm::ortho(left, right, bottom, top, nearPlane, farPlane);
  return projection * view;
}

Ray OrthographicCamera::screenPointToRay(Vec2f viewportPos,
                                         Vec2f viewportSize) const noexcept {
  Vec2f ndc = viewportToNdc(viewportPos, viewportSize);
  Ray ray;
  ray.origin = Vec3f(center.x + ndc.x * halfHeight * aspect,
                     center.y + ndc.y * halfHeight, kOrthoEyeZ);
  ray.direction = Vec3f(0.0f, 0.0f, -1.0f);
  return ray;
}

Ray PerspectiveCamera::screenPointToRay(Vec2f viewportPos,
                                        Vec2f viewportSize) const noexcept {
  Vec2f ndc = viewportToNdc(viewportPos, viewportSize);

  Vec3f forward = target - position;
  forward = glm::length(forward) > 0.0f ? glm::normalize(forward)
                                        : Vec3f(0.0f, 0.0f, -1.0f);
  Vec3f right = glm::cross(forward, up);
  right = glm::length(right) > 0.0f ? glm::normalize(right)
                                    : Vec3f(1.0f, 0.0f, 0.0f);
  Vec3f trueUp = glm::cross(right, forward);

  float tanHalfFov = std::tan(fovYRadians * 0.5f);
  Vec3f dir = forward + right * (ndc.x * tanHalfFov * aspect) +
              trueUp * (ndc.y * tanHalfFov);

  Ray ray;
  ray.origin = position;
  ray.direction = glm::normalize(dir);
  return ray;
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
