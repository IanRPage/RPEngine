#ifndef RPENGINE_RENDER_IMGUICONTROLLER_HPP
#define RPENGINE_RENDER_IMGUICONTROLLER_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

struct GLFWwindow;

namespace gui {

enum class CameraMode { Orthographic2D, Perspective3D };

class ImguiController {
 public:
  static constexpr float kSidebarWidth = 340.0f;

  explicit ImguiController(GLFWwindow* window);
  ~ImguiController();

  ImguiController(const ImguiController&) = delete;
  ImguiController& operator=(const ImguiController&) = delete;

  void beginFrame() noexcept;
  void renderPanels(float frameTimeMs, size_t liveBodyCount,
                    size_t sleepingBodyCount = 0);
  void endFrame() noexcept;

  CameraMode cameraMode() const noexcept { return cameraMode_; }

  bool consumeSpawnSphereRequest() noexcept;
  bool consumeSpawnBoxRequest() noexcept;
  bool consumeSpawnCapsuleRequest() noexcept;
  bool consumeSpawnCompoundRequest() noexcept;
  bool consumeResetRequest() noexcept;
  bool consumeWakeAllRequest() noexcept;
  bool consumeDeleteSelectedRequest() noexcept;

  void setSelectedBody(std::optional<uint32_t> bodyIndex) noexcept {
    selectedBody_ = bodyIndex;
  }
  std::optional<uint32_t> selectedBody() const noexcept {
    return selectedBody_;
  }

  bool consumeScreenshotRequest(std::string& path);
  void setLastScreenshotStatus(std::string status) {
    lastScreenshotStatus_ = std::move(status);
  }

 private:
  void renderCameraControls();
  void renderFrameStats(float frameTimeMs, size_t liveBodyCount,
                        size_t sleepingBodyCount);
  void renderSpawnControls();
  void renderScreenshotControls();

  CameraMode cameraMode_ = CameraMode::Perspective3D;
  bool spawnSphereRequested_ = false;
  bool spawnBoxRequested_ = false;
  bool spawnCapsuleRequested_ = false;
  bool spawnCompoundRequested_ = false;
  bool resetRequested_ = false;
  bool wakeAllRequested_ = false;
  bool deleteSelectedRequested_ = false;

  std::optional<uint32_t> selectedBody_;

  bool screenshotRequested_ = false;
  std::string screenshotPath_;
  char screenshotPathBuffer_[256] = "screenshot.png";
  std::string lastScreenshotStatus_;
};

}  // namespace gui

#endif
