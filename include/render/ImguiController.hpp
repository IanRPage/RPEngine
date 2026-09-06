#ifndef RPENGINE_RENDER_IMGUICONTROLLER_HPP
#define RPENGINE_RENDER_IMGUICONTROLLER_HPP

#include <cstddef>

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
  bool consumeResetRequest() noexcept;

 private:
  void renderCameraControls();
  void renderFrameStats(float frameTimeMs, size_t liveBodyCount);
  void renderSpawnControls();

  CameraMode cameraMode_ = CameraMode::Orthographic2D;
  bool spawnSphereRequested_ = false;
  bool spawnBoxRequested_ = false;
  bool resetRequested_ = false;
};

}  // namespace render

#endif
