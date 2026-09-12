#ifndef RPENGINE_RENDER_IMGUICONTROLLER_HPP
#define RPENGINE_RENDER_IMGUICONTROLLER_HPP

#include <cstddef>
#include <string>

namespace render {

enum class CameraMode { Orthographic2D, Perspective3D };

class ImguiController {
 public:
  static constexpr float kSidebarWidth = 340.0f;

  void beginFrame() noexcept;
  void renderPanels(float frameTimeMs, size_t liveBodyCount);
  void endFrame() noexcept;

  CameraMode cameraMode() const noexcept { return cameraMode_; }

  bool consumeSpawnSphereRequest() noexcept;
  bool consumeSpawnBoxRequest() noexcept;
  bool consumeSpawnCapsuleRequest() noexcept;
  bool consumeResetRequest() noexcept;

  bool consumeScreenshotRequest(std::string& path);
  void setLastScreenshotStatus(std::string status) {
    lastScreenshotStatus_ = std::move(status);
  }

 private:
  void renderCameraControls();
  void renderFrameStats(float frameTimeMs, size_t liveBodyCount);
  void renderSpawnControls();
  void renderScreenshotControls();

  CameraMode cameraMode_ = CameraMode::Perspective3D;
  bool spawnSphereRequested_ = false;
  bool spawnBoxRequested_ = false;
  bool spawnCapsuleRequested_ = false;
  bool resetRequested_ = false;

  bool screenshotRequested_ = false;
  std::string screenshotPath_;
  char screenshotPathBuffer_[256] = "screenshot.png";
  std::string lastScreenshotStatus_;
};

}  // namespace render

#endif
