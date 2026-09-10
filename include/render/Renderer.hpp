#ifndef RPENGINE_RENDER_RENDERER_HPP
#define RPENGINE_RENDER_RENDERER_HPP

#include <render/Camera.hpp>
#include <render/HullMeshCache.hpp>
#include <render/ImguiController.hpp>
#include <render/InstancedBatch.hpp>
#include <render/Mesh.hpp>
#include <render/RenderableStore.hpp>
#include <render/ShaderProgram.hpp>
#include <render/Window.hpp>
#include <sim/Simulator.hpp>

#include <string>

namespace render {

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
  void captureDebugScreenshot();

  Simulator& sim_;
  RenderableStore& renderables_;

  Window window_;
  ShaderProgram shader_;

  Mesh sphereMesh_;
  Mesh boxMesh_;
  Mesh capsuleMesh_;
  Mesh quadMesh_;
  Mesh circleMesh_;
  Mesh stadiumMesh_;

  InstancedBatch sphereBatch_;
  InstancedBatch boxBatch_;
  InstancedBatch capsuleBatch_;
  InstancedBatch quadBatch_;
  InstancedBatch circleBatch_;
  InstancedBatch stadiumBatch_;
  InstancedBatch hullBatch_;
  HullMeshCache hullMeshCache_;

  ImguiController imguiCtrl_;

  OrthographicCamera orthoCamera_;
  FreeFlyCameraController flyCamera_;

  bool hasLastMouseLookPos_ = false;
  Vec2f lastMouseLookPos_{0.0f, 0.0f};
  bool mouseCaptured_ = false;

  double lastFrameTimestamp_ = 0.0;
  double lastAdvanceTimestamp_ = 0.0;
};

}  // namespace render

#endif
