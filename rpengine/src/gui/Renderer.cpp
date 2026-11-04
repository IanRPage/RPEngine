#include <gui/Renderer.hpp>

#include <math/Rotation.hpp>
#include <gui/AssetPaths.hpp>
#include <render/MeshLibrary.hpp>

#include <imgui.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

namespace gui {

namespace {

Vec4f randomColor() noexcept {
  auto channel = []() {
    return static_cast<float>(std::rand() % 156 + 100) / 255.0f;
  };
  return Vec4f(channel(), channel(), channel(), 1.0f);
}

constexpr float kClickSlopPixels = 4.0f;
constexpr float kOrthoPickHeight = 1.0e4f;
const Vec4f kSelectionTint{1.0f, 0.85f, 0.2f, 1.0f};

std::string expandTilde(const std::string& path) {
  if (path.empty() || path[0] != '~') { return path; }
  if (path.size() > 1 && path[1] != '/') { return path; }
  const char* home = std::getenv("HOME");
  if (home == nullptr) { return path; }
  return std::string(home) + path.substr(1);
}

Transform interpolatedTransform(const BodyStore& bodies, BodyHandle handle,
                                float alpha) noexcept {
  return Transform{
      glm::mix(bodies.prevPosition(handle), bodies.position(handle), alpha),
      nlerp(bodies.prevOrientation(handle), bodies.orientation(handle), alpha)};
}

}  // namespace

Renderer::Renderer(Simulator& sim, RenderableStore& renderables,
                   const Options& options)
    : sim_(sim),
      renderables_(renderables),
      window_(options.width, options.height, options.title),
      shader_(render::ShaderProgram::fromFiles(
          std::filesystem::path(kAssetsDir) / "shaders" / "instanced.vert",
          std::filesystem::path(kAssetsDir) / "shaders" / "instanced.frag")),
      sphereMesh_(render::buildUnitIcosphere()),
      boxMesh_(render::buildUnitBox()),
      capsuleMesh_(render::buildUnitCapsule()),
      quadMesh_(render::buildUnitQuad()),
      circleMesh_(render::buildUnitCircle()),
      stadiumMesh_(render::buildUnitStadium()),
      imguiCtrl_(window_.handle()) {
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

float Renderer::worldViewAspect() const noexcept {
  int fbWidth = window_.framebufferWidth();
  int fbHeight = window_.framebufferHeight();
  float scaleX = static_cast<float>(fbWidth) /
                 static_cast<float>(std::max(window_.width(), 1));
  int sidebarPx = static_cast<int>(ImguiController::kSidebarWidth * scaleX);
  int viewportWidth = std::max(fbWidth - sidebarPx, 1);
  return static_cast<float>(viewportWidth) /
         static_cast<float>(std::max(fbHeight, 1));
}

Vec2f Renderer::worldViewSize() const noexcept {
  return Vec2f(std::max(static_cast<float>(window_.width()) -
                            ImguiController::kSidebarWidth,
                        1.0f),
               std::max(static_cast<float>(window_.height()), 1.0f));
}

bool Renderer::pickBodyAt(Vec2f viewportPos) noexcept {
  Vec2f viewportSize = worldViewSize();
  float aspect = worldViewAspect();

  Ray ray;
  float maxDistance = 0.0f;
  if (imguiCtrl_.cameraMode() == CameraMode::Orthographic2D) {
    orthoCamera_.aspect = aspect;
    render::Ray r = orthoCamera_.screenPointToRay(viewportPos, viewportSize);
    ray.origin = Vec3f(r.origin.x, r.origin.y, kOrthoPickHeight);
    ray.direction = r.direction;
    maxDistance = 2.0f * kOrthoPickHeight;
  } else {
    render::PerspectiveCamera camera = flyCamera_.toCamera(aspect);
    render::Ray r = camera.screenPointToRay(viewportPos, viewportSize);
    ray.origin = r.origin;
    ray.direction = r.direction;
    maxDistance = camera.farPlane;
  }

  std::optional<WorldRayHit> hit =
      sim_.world().raycastClosest(ray, maxDistance);
  const BodyStore& bodies = sim_.world().bodies();
  if (hit && bodies.invMass(hit->body) > 0.0f) {
    selectedBody_ = hit->body;
    return true;
  }
  selectedBody_ = BodyHandle{};
  return false;
}

void Renderer::refreshSelection() noexcept {
  const BodyStore& bodies = sim_.world().bodies();
  if (selectedBody_.valid() && !bodies.isLive(selectedBody_)) {
    selectedBody_ = BodyHandle{};
  }
  if (selectedBody_.valid()) {
    imguiCtrl_.setSelectedBody(selectedBody_.index);
  } else {
    imguiCtrl_.setSelectedBody(std::nullopt);
  }
}

Vec4f Renderer::displayColor(BodyHandle handle) const noexcept {
  Vec4f color = renderables_.colorOr(handle);
  if (selectedBody_.valid() && handle == selectedBody_) {
    color = glm::mix(color, kSelectionTint, 0.6f);
  }
  return color;
}

void Renderer::handleWorldViewInput(float dt) noexcept {
  ImGuiIO& io = ImGui::GetIO();
  bool overWorldView =
      !io.WantCaptureMouse && io.MousePos.x >= ImguiController::kSidebarWidth;
  GLFWwindow* handle = window_.handle();
  Vec2f center =
      Vec2f(ImguiController::kSidebarWidth, 0.0f) + worldViewSize() * 0.5f;

  if (imguiCtrl_.cameraMode() == CameraMode::Orthographic2D) {
    if (mouseCaptured_) {
      glfwSetInputMode(handle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      glfwSetCursorPos(handle, center.x, center.y);
      mouseCaptured_ = false;
    }
    hasLastMouseLookPos_ = false;
    Vec2f mousePos(io.MousePos.x, io.MousePos.y);
    if (overWorldView && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      orthoClickPending_ = true;
      orthoClickStart_ = mousePos;
    }
    if (orthoClickPending_ &&
        glm::length(mousePos - orthoClickStart_) > kClickSlopPixels) {
      orthoClickPending_ = false;
    }
    if (orthoClickPending_ && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
      orthoClickPending_ = false;
      pickBodyAt(mousePos - Vec2f(ImguiController::kSidebarWidth, 0.0f));
    }
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

  orthoClickPending_ = false;

  if (mouseCaptured_ && glfwGetKey(handle, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetInputMode(handle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetCursorPos(handle, center.x, center.y);
    mouseCaptured_ = false;
    hasLastMouseLookPos_ = false;
  } else if (mouseCaptured_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    pickBodyAt(worldViewSize() * 0.5f);
  } else if (!mouseCaptured_ && overWorldView &&
             ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    glfwSetInputMode(handle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPos(handle, center.x, center.y);
    mouseCaptured_ = true;
    hasLastMouseLookPos_ = false;
  }

  if (!mouseCaptured_) { return; }

  double rawX = 0.0, rawY = 0.0;
  glfwGetCursorPos(handle, &rawX, &rawY);
  Vec2f mousePos(static_cast<float>(rawX), static_cast<float>(rawY));
  if (hasLastMouseLookPos_) {
    flyCamera_.onMouseLook(mousePos - lastMouseLookPos_);
  }
  lastMouseLookPos_ = mousePos;
  hasLastMouseLookPos_ = true;

  constexpr float kMoveSpeed = 8.0f;  // world units / second
  float step = kMoveSpeed * dt;
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
  if (glfwGetKey(handle, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
      glfwGetKey(handle, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS) {
    flyCamera_.moveUp(-step);
  }
}

void Renderer::handlePendingSpawnRequests() {
  World& world = sim_.world();

  if (imguiCtrl_.consumeSpawnSphereRequest()) {
    float x = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float z = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 7.0f;
    BodyHandle handle = world.createDynamicBody(
        SphereShape{0.5f},
        Transform{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 1.0f, 0.5f,
        0.6f);
    renderables_.setColor(handle, randomColor());
  }

  if (imguiCtrl_.consumeSpawnBoxRequest()) {
    float x = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float z = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 7.0f;
    BodyHandle handle = world.createDynamicBody(
        BoxShape{Vec3f(0.5f, 0.5f, 0.5f)},
        Transform{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 1.0f, 0.5f,
        0.4f);
    renderables_.setColor(handle, randomColor());
  }

  if (imguiCtrl_.consumeSpawnCapsuleRequest()) {
    float x = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float z = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 7.0f;
    BodyHandle handle = world.createDynamicBody(
        CapsuleShape{0.4f, 0.5f},
        Transform{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 1.0f, 0.5f,
        0.5f);
    renderables_.setColor(handle, randomColor());
  }

  if (imguiCtrl_.consumeSpawnCompoundRequest()) {
    float x = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float z = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float y = static_cast<float>(std::rand() % 300) / 100.0f + 7.0f;
    std::vector<CompoundChild> children;
    children.push_back(CompoundChild{BoxShape{Vec3f(0.15f, 0.6f, 0.15f)},
                                     Transform{}, 1.0f});
    children.push_back(
        CompoundChild{BoxShape{Vec3f(0.5f, 0.2f, 0.25f)},
                      Transform{Vec3f(0.0f, 0.8f, 0.0f),
                                Quatf(1.0f, 0.0f, 0.0f, 0.0f)},
                      3.0f});
    children.push_back(CompoundChild{SphereShape{0.25f},
                                     Transform{Vec3f(0.0f, -0.75f, 0.0f),
                                               Quatf(1.0f, 0.0f, 0.0f, 0.0f)},
                                     1.0f});
    BodyHandle handle = world.createDynamicBody(
        CompoundShape{std::move(children)}.recentered(),
        Transform{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 2.0f, 0.5f,
        0.3f);
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

  if (imguiCtrl_.consumeWakeAllRequest()) { world.wakeAll(); }

  bool deleteKeyPressed = !ImGui::GetIO().WantCaptureKeyboard &&
                          ImGui::IsKeyPressed(ImGuiKey_Backspace);
  if (imguiCtrl_.consumeDeleteSelectedRequest() || deleteKeyPressed) {
    const BodyStore& store = world.bodies();
    if (selectedBody_.valid() && store.isLive(selectedBody_) &&
        store.invMass(selectedBody_) > 0.0f) {
      world.removeBody(selectedBody_);
      selectedBody_ = BodyHandle{};
    }
  }

  refreshSelection();
}

void Renderer::drawShapeBodies(float alpha, const Mat4f& viewProjection) {
  sphereBatch_.begin();
  boxBatch_.begin();
  capsuleBatch_.begin();
  quadBatch_.begin();
  circleBatch_.begin();
  stadiumBatch_.begin();

  const BodyStore& bodies = sim_.world().bodies();
  bool is2D = imguiCtrl_.cameraMode() == CameraMode::Orthographic2D;

  auto addLeaf = [&](const ShapeVariant& leaf, const Transform& t,
                     const Vec4f& color) {
    Vec3f position = t.position;
    if (is2D) { position.z = 0.0f; }
    render::InstanceData instance{position, t.orientation, Vec3f(1.0f), color};

    std::visit(
        [&](const auto& s) {
          using T = std::decay_t<decltype(s)>;
          if constexpr (std::is_same_v<T, SphereShape>) {
            instance.scale = Vec3f(s.radius);
            (is2D ? circleBatch_ : sphereBatch_).add(instance);
          } else if constexpr (std::is_same_v<T, BoxShape>) {
            instance.scale = s.halfExtents;
            if (!is2D && instance.scale.z <= 0.0f) {
              instance.scale.z = std::min(instance.scale.x, instance.scale.y);
            }
            (is2D ? quadBatch_ : boxBatch_).add(instance);
          } else if constexpr (std::is_same_v<T, CapsuleShape>) {
            instance.scale = Vec3f(s.radius, s.halfHeight, s.radius);
            (is2D ? stadiumBatch_ : capsuleBatch_).add(instance);
          }
        },
        leaf);
  };

  for (BodyHandle handle : bodies.liveHandles()) {
    const ShapeVariant& shape = bodies.shape(handle);
    if (std::holds_alternative<ConvexHullShape>(shape)) { continue; }

    Transform bodyTransform = interpolatedTransform(bodies, handle, alpha);
    Vec4f color = displayColor(handle);

    if (const auto* compound = std::get_if<CompoundShape>(&shape)) {
      for (const CompoundChild& child : compound->children) {
        addLeaf(child.shape, compose(bodyTransform, child.localTransform),
                color);
      }
    } else {
      addLeaf(shape, bodyTransform, color);
    }
  }

  sphereBatch_.render(sphereMesh_, shader_, viewProjection);
  boxBatch_.render(boxMesh_, shader_, viewProjection);
  capsuleBatch_.render(capsuleMesh_, shader_, viewProjection);
  quadBatch_.render(quadMesh_, shader_, viewProjection);
  circleBatch_.render(circleMesh_, shader_, viewProjection);
  stadiumBatch_.render(stadiumMesh_, shader_, viewProjection);
}

void Renderer::drawHullBodies(float alpha, const Mat4f& viewProjection) {
  const BodyStore& bodies = sim_.world().bodies();
  bool is2D = imguiCtrl_.cameraMode() == CameraMode::Orthographic2D;

  auto drawHull = [&](const render::Mesh& mesh, const Transform& t,
                      const Vec4f& color) {
    Vec3f position = t.position;
    if (is2D) { position.z = 0.0f; }
    hullBatch_.begin();
    hullBatch_.add(
        render::InstanceData{position, t.orientation, Vec3f(1.0f), color});
    hullBatch_.render(mesh, shader_, viewProjection);
  };

  for (BodyHandle handle : bodies.liveHandles()) {
    const ShapeVariant& shape = bodies.shape(handle);
    if (const auto* hull = std::get_if<ConvexHullShape>(&shape)) {
      drawHull(hullMeshCache_.getOrBuild(handle, *hull),
               interpolatedTransform(bodies, handle, alpha),
               displayColor(handle));
      continue;
    }

    const auto* compound = std::get_if<CompoundShape>(&shape);
    if (compound == nullptr) { continue; }

    for (std::size_t i = 0; i < compound->children.size(); ++i) {
      const CompoundChild& child = compound->children[i];
      const auto* childHull = std::get_if<ConvexHullShape>(&child.shape);
      if (childHull == nullptr) { continue; }
      Transform t = compose(interpolatedTransform(bodies, handle, alpha),
                            child.localTransform);
      drawHull(hullMeshCache_.getOrBuild(handle, static_cast<uint32_t>(i),
                                         *childHull),
               t, displayColor(handle));
    }
  }

  hullMeshCache_.pruneDead(bodies);
}

bool Renderer::writeFramebufferToPng(const std::string& path) const {
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

  return stbi_write_png(path.c_str(), width, height, 3, flipped.data(),
                        static_cast<int>(rowBytes)) != 0;
}

void Renderer::handleScreenshotRequest() {
  if (screenshotPending_) {
    screenshotPending_ = false;
    std::string resolvedPath = expandTilde(pendingScreenshotPath_);
    bool ok = writeFramebufferToPng(resolvedPath);
    imguiCtrl_.setLastScreenshotStatus(ok ? ("Saved to " + resolvedPath)
                                          : ("Failed to save to " + resolvedPath));
    std::fprintf(stderr, "[RPEngine] %s screenshot %s\n",
                ok ? "wrote" : "FAILED to write", resolvedPath.c_str());
  }

  std::string path;
  if (imguiCtrl_.consumeScreenshotRequest(path)) {
    pendingScreenshotPath_ = path;
    screenshotPending_ = true;
  }
}

void Renderer::captureScreenshot() {
  static const char* afterEnv = std::getenv("RPENGINE_SCREENSHOT_AFTER");
  static const char* pathEnv = std::getenv("RPENGINE_SCREENSHOT_PATH");
  if (afterEnv == nullptr || pathEnv == nullptr) { return; }

  static int targetFrame = std::atoi(afterEnv);
  static int frameCounter = 0;
  ++frameCounter;
  if (frameCounter != targetFrame) { return; }

  std::string resolvedPath = expandTilde(pathEnv);
  bool ok = writeFramebufferToPng(resolvedPath);
  std::fprintf(stderr, "[RPEngine] %s debug screenshot %s\n",
              ok ? "wrote" : "FAILED to write", resolvedPath.c_str());
  std::exit(ok ? 0 : 1);
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
  float aspect = worldViewAspect();

  shader_.bind();
  shader_.setUniform("uLightDirection", Vec3f(-0.3f, -1.0f, -0.2f));
  shader_.setUniform("uAmbientStrength", 0.3f);

  float alpha = std::min(sim_.interpolationAlpha(), 1.0f);
  Mat4f viewProjection = currentViewProjection(aspect);

  drawShapeBodies(alpha, viewProjection);
  drawHullBodies(alpha, viewProjection);

  imguiCtrl_.renderPanels(frameTimeMs, sim_.world().bodies().size(),
                          sim_.world().sleepingBodyCount());
  if (mouseCaptured_) {
    Vec2f c =
        Vec2f(ImguiController::kSidebarWidth, 0.0f) + worldViewSize() * 0.5f;
    constexpr float kArm = 5.0f;
    ImU32 col = IM_COL32(255, 255, 255, 220);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    dl->AddLine(ImVec2(c.x - kArm, c.y), ImVec2(c.x + kArm, c.y), col, 1.5f);
    dl->AddLine(ImVec2(c.x, c.y - kArm), ImVec2(c.x, c.y + kArm), col, 1.5f);
  }
  imguiCtrl_.endFrame();

  handleScreenshotRequest();
  captureScreenshot();
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

}  // namespace gui
