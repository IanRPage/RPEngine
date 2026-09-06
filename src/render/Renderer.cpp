#include <render/Renderer.hpp>

#include <math/Rotation.hpp>
#include <render/AssetPaths.hpp>
#include <render/MeshLibrary.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <type_traits>
#include <variant>
#include <vector>

namespace render {

namespace {

Vec4f randomColor() noexcept {
  auto channel = []() {
    return static_cast<float>(std::rand() % 156 + 100) / 255.0f;
  };
  return Vec4f(channel(), channel(), channel(), 1.0f);
}

}  // namespace

Renderer::Renderer(Simulator& sim, RenderableStore& renderables,
                   const Options& options)
    : sim_(sim),
      renderables_(renderables),
      window_(options.width, options.height, options.title),
      shader_(ShaderProgram::fromFiles(
          std::filesystem::path(kAssetsDir) / "shaders" / "instanced.vert",
          std::filesystem::path(kAssetsDir) / "shaders" / "instanced.frag")),
      sphereMesh_(buildUnitIcosphere()),
      boxMesh_(buildUnitBox()),
      capsuleMesh_(buildUnitCapsule()),
      quadMesh_(buildUnitQuad()),
      circleMesh_(buildUnitCircle()) {
  glEnable(GL_DEPTH_TEST);
  lastFrameTimestamp_ = glfwGetTime();
  lastAdvanceTimestamp_ = lastFrameTimestamp_;
}

bool Renderer::isOpen() const noexcept { return !window_.shouldClose(); }

void Renderer::pollAndHandleEvents() noexcept { window_.pollEvents(); }

Mat4f Renderer::currentViewProjection(float aspect) noexcept {
  if (imguiCtrl_.cameraMode() == CameraMode::Orthographic2D) {
    orthoCamera_.aspect = aspect;
    return orthoCamera_.viewProjection();
  }
  return flyCamera_.toCamera(aspect).viewProjection();
}

void Renderer::handleWorldViewInput(float dt) noexcept {
  ImGuiIO& io = ImGui::GetIO();
  bool overWorldView =
      !io.WantCaptureMouse && io.MousePos.x >= ImguiController::kSidebarWidth;

  if (imguiCtrl_.cameraMode() == CameraMode::Orthographic2D) {
    if (overWorldView && io.MouseDown[ImGuiMouseButton_Left]) {
      float viewportHeight =
          std::max(static_cast<float>(window_.height()), 1.0f);
      float worldPerPixel = (2.0f * orthoCamera_.halfHeight) / viewportHeight;
      Vec2f worldDelta(io.MouseDelta.x * worldPerPixel,
                       -io.MouseDelta.y * worldPerPixel);
      orthoCamera_.center -= worldDelta;
    }
    return;
  }

  if (overWorldView && io.MouseDown[ImGuiMouseButton_Left]) {
    flyCamera_.onMouseLook(Vec2f(io.MouseDelta.x, io.MouseDelta.y));
  }
  if (!io.WantCaptureKeyboard) {
    constexpr float kMoveSpeed = 8.0f;  // world units / second
    float step = kMoveSpeed * dt;
    GLFWwindow* handle = window_.handle();
    if (glfwGetKey(handle, GLFW_KEY_W) == GLFW_PRESS) {
      flyCamera_.moveForward(step);
    }
    if (glfwGetKey(handle, GLFW_KEY_S) == GLFW_PRESS) {
      flyCamera_.moveForward(-step);
    }
    if (glfwGetKey(handle, GLFW_KEY_D) == GLFW_PRESS) {
      flyCamera_.moveRight(step);
    }
    if (glfwGetKey(handle, GLFW_KEY_A) == GLFW_PRESS) {
      flyCamera_.moveRight(-step);
    }
    if (glfwGetKey(handle, GLFW_KEY_SPACE) == GLFW_PRESS) {
      flyCamera_.moveUp(step);
    }
    if (glfwGetKey(handle, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
      flyCamera_.moveUp(-step);
    }
  }
}

void Renderer::handlePendingSpawnRequests() {
  World& world = sim_.world();

  if (imguiCtrl_.consumeSpawnSphereRequest()) {
    float x = static_cast<float>(std::rand() % 800 - 400) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 6.0f;
    BodyHandle handle = world.createDynamicBody(
        SphereShape{0.5f},
        Transform{Vec3f(x, y, 0.0f), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 1.0f, 0.5f,
        0.6f, true);
    renderables_.setColor(handle, randomColor());
  }

  if (imguiCtrl_.consumeSpawnBoxRequest()) {
    float x = static_cast<float>(std::rand() % 800 - 400) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 6.0f;
    BodyHandle handle = world.createDynamicBody(
        BoxShape{Vec3f(0.5f, 0.5f, 0.5f)},
        Transform{Vec3f(x, y, 0.0f), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 1.0f, 0.5f,
        0.4f, true);
    renderables_.setColor(handle, randomColor());
  }

  if (imguiCtrl_.consumeResetRequest()) {
    BodyStore& store = world.bodies();
    std::vector<BodyHandle> dynamicHandles;
    for (BodyHandle h : store.liveHandles()) {
      if (store.invMass(h) > 0.0f) { dynamicHandles.push_back(h); }
    }
    for (BodyHandle h : dynamicHandles) { store.removeBody(h); }
  }
}

void Renderer::drawShapeBodies(float alpha, const Mat4f& viewProjection) {
  sphereBatch_.begin();
  boxBatch_.begin();
  capsuleBatch_.begin();
  quadBatch_.begin();
  circleBatch_.begin();

  const BodyStore& bodies = sim_.world().bodies();
  bool is2D = imguiCtrl_.cameraMode() == CameraMode::Orthographic2D;

  for (BodyHandle handle : bodies.liveHandles()) {
    const ShapeVariant& shape = bodies.shape(handle);
    if (std::holds_alternative<ConvexHullShape>(shape)) { continue; }

    Vec3f position =
        glm::mix(bodies.prevPosition(handle), bodies.position(handle), alpha);
    Quatf orientation = nlerp(bodies.prevOrientation(handle),
                              bodies.orientation(handle), alpha);
    Vec4f color = renderables_.colorOr(handle);

    InstanceData instance{position, orientation, Vec3f(1.0f), color};

    std::visit(
        [&](const auto& s) {
          using T = std::decay_t<decltype(s)>;
          if constexpr (std::is_same_v<T, SphereShape>) {
            instance.scale = Vec3f(s.radius);
            (is2D ? circleBatch_ : sphereBatch_).add(instance);
          } else if constexpr (std::is_same_v<T, BoxShape>) {
            instance.scale = s.halfExtents;
            if (!is2D) {
              float minVisualHalfThickness =
                  std::min(instance.scale.x, instance.scale.y);
              instance.scale.z =
                  std::max(instance.scale.z, minVisualHalfThickness);
            }
            (is2D ? quadBatch_ : boxBatch_).add(instance);
          } else if constexpr (std::is_same_v<T, CapsuleShape>) {
            instance.scale = Vec3f(s.radius, s.halfHeight, s.radius);
            capsuleBatch_.add(instance);
          }
        },
        shape);
  }

  sphereBatch_.render(sphereMesh_, shader_, viewProjection);
  boxBatch_.render(boxMesh_, shader_, viewProjection);
  capsuleBatch_.render(capsuleMesh_, shader_, viewProjection);
  quadBatch_.render(quadMesh_, shader_, viewProjection);
  circleBatch_.render(circleMesh_, shader_, viewProjection);
}

void Renderer::drawHullBodies(float alpha, const Mat4f& viewProjection) {
  const BodyStore& bodies = sim_.world().bodies();

  for (BodyHandle handle : bodies.liveHandles()) {
    const ShapeVariant& shape = bodies.shape(handle);
    const auto* hull = std::get_if<ConvexHullShape>(&shape);
    if (hull == nullptr) { continue; }

    Vec3f position =
        glm::mix(bodies.prevPosition(handle), bodies.position(handle), alpha);
    Quatf orientation = nlerp(bodies.prevOrientation(handle),
                              bodies.orientation(handle), alpha);
    Vec4f color = renderables_.colorOr(handle);

    const Mesh& mesh = hullMeshCache_.getOrBuild(handle, *hull);
    hullBatch_.begin();
    hullBatch_.add(InstanceData{position, orientation, Vec3f(1.0f), color});
    hullBatch_.render(mesh, shader_, viewProjection);
  }

  hullMeshCache_.pruneDead(bodies);
}

void Renderer::captureDebugScreenshot() {
  static const char* afterEnv = std::getenv("RPENGINE_SCREENSHOT_AFTER");
  static const char* pathEnv = std::getenv("RPENGINE_SCREENSHOT_PATH");
  if (afterEnv == nullptr || pathEnv == nullptr) { return; }

  static int targetFrame = std::atoi(afterEnv);
  static int frameCounter = 0;
  ++frameCounter;
  if (frameCounter != targetFrame) { return; }

  int width = window_.framebufferWidth();
  int height = window_.framebufferHeight();
  std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  size_t rowBytes = static_cast<size_t>(width) * 3;
  std::vector<unsigned char> flipped(pixels.size());
  for (int y = 0; y < height; ++y) {
    std::memcpy(&flipped[static_cast<size_t>(y) * rowBytes],
                &pixels[static_cast<size_t>(height - 1 - y) * rowBytes],
                rowBytes);
  }

  std::ofstream out(pathEnv, std::ios::binary);
  if (out) {
    out << "P6\n" << width << ' ' << height << "\n255\n";
    out.write(reinterpret_cast<const char*>(flipped.data()),
              static_cast<std::streamsize>(flipped.size()));
    out.close();
    std::fprintf(stderr, "[RPEngine] wrote debug screenshot to %s\n", pathEnv);
  }
  std::exit(0);
}

void Renderer::drawFrame() {
  double now = glfwGetTime();
  float frameTimeMs = static_cast<float>((now - lastFrameTimestamp_) * 1000.0);
  lastFrameTimestamp_ = now;
  float dt = frameTimeMs / 1000.0f;

  imguiCtrl_.beginFrame();
  handleWorldViewInput(dt);
  handlePendingSpawnRequests();

  int fbWidth = window_.framebufferWidth();
  int fbHeight = window_.framebufferHeight();
  float scaleX = static_cast<float>(fbWidth) /
                 static_cast<float>(std::max(window_.width(), 1));
  int sidebarPx = static_cast<int>(ImguiController::kSidebarWidth * scaleX);
  int viewportX = std::min(sidebarPx, fbWidth);
  int viewportWidth = std::max(fbWidth - sidebarPx, 1);

  glViewport(0, 0, fbWidth, fbHeight);
  glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glViewport(viewportX, 0, viewportWidth, fbHeight);
  float aspect = static_cast<float>(viewportWidth) /
                 static_cast<float>(std::max(fbHeight, 1));

  shader_.bind();
  shader_.setUniform("uLightDirection", Vec3f(-0.3f, -1.0f, -0.2f));
  shader_.setUniform("uAmbientStrength", 0.3f);

  float alpha = std::min(sim_.interpolationAlpha(), 1.0f);
  Mat4f viewProjection = currentViewProjection(aspect);

  drawShapeBodies(alpha, viewProjection);
  drawHullBodies(alpha, viewProjection);

  imguiCtrl_.renderPanels(frameTimeMs, sim_.world().bodies().size());
  imguiCtrl_.endFrame();

  captureDebugScreenshot();
  window_.swapBuffers();
}

void Renderer::mainLoop() {
  while (isOpen()) {
    pollAndHandleEvents();

    double now = glfwGetTime();
    float dt = static_cast<float>(now - lastAdvanceTimestamp_);
    lastAdvanceTimestamp_ = now;

    sim_.advance(dt);
    drawFrame();
  }
}

}  // namespace render
