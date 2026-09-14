#ifndef RPENGINE_RENDER_RENDERER_HPP
#define RPENGINE_RENDER_RENDERER_HPP

#include <gui/HullMeshCache.hpp>
#include <gui/ImguiController.hpp>
#include <gui/RenderableStore.hpp>
#include <render/Camera.hpp>
#include <render/InstancedBatch.hpp>
#include <render/Mesh.hpp>
#include <render/ShaderProgram.hpp>
#include <render/Window.hpp>
#include <sim/Simulator.hpp>

#include <string>

namespace gui {

class Renderer {
 public:
  struct Options {
    int width = 1280;
    int height = 720;
    std::string title = "RPEngine";
  };

  Renderer(Simulator& sim, RenderableStore& renderables,
           const Options& options);
  Renderer(Simulator& sim, RenderableStore& renderables)
      : Renderer(sim, renderables, Options()) {}

  bool isOpen() const noexcept;
  void pollAndHandleEvents() noexcept;
  void drawFrame();
  void mainLoop();

 private:
  Mat4f currentViewProjection(float aspect) noexcept;
  void handleWorldViewInput(float dt) noexcept;
  void drawShapeBodies(float alpha, const Mat4f& viewProjection);
  void drawHullBodies(float alpha, const Mat4f& viewProjection);
  void handlePendingSpawnRequests();
  void handleScreenshotRequest();
  void captureScreenshot();
  bool writeFramebufferToPng(const std::string& path) const;

  Simulator& sim_;
  RenderableStore& renderables_;

  render::Window window_;
  render::ShaderProgram shader_;

  render::Mesh sphereMesh_;
  render::Mesh boxMesh_;
  render::Mesh capsuleMesh_;
  render::Mesh quadMesh_;
  render::Mesh circleMesh_;
  render::Mesh stadiumMesh_;

  render::InstancedBatch sphereBatch_;
  render::InstancedBatch boxBatch_;
  render::InstancedBatch capsuleBatch_;
  render::InstancedBatch quadBatch_;
  render::InstancedBatch circleBatch_;
  render::InstancedBatch stadiumBatch_;
  render::InstancedBatch hullBatch_;
  HullMeshCache hullMeshCache_;

  ImguiController imguiCtrl_;

  render::OrthographicCamera orthoCamera_;
  render::FreeFlyCameraController flyCamera_;

  bool hasLastMouseLookPos_ = false;
  Vec2f lastMouseLookPos_{0.0f, 0.0f};
  bool mouseCaptured_ = false;

  bool screenshotPending_ = false;
  std::string pendingScreenshotPath_;

  double lastFrameTimestamp_ = 0.0;
  double lastAdvanceTimestamp_ = 0.0;
};

}  // namespace gui

#endif
