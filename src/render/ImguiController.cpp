#include <render/ImguiController.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace render {

void ImguiController::beginFrame() noexcept {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void ImguiController::renderPanels(float frameTimeMs, size_t liveBodyCount) {
  const ImGuiIO& io = ImGui::GetIO();

  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
  ImGui::SetNextWindowSize(ImVec2(kSidebarWidth, io.DisplaySize.y));
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoCollapse |
                           ImGuiWindowFlags_NoBringToFrontOnFocus;
  ImGui::Begin("RPEngine", nullptr, flags);

  if (ImGui::CollapsingHeader("Frame Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
    renderFrameStats(frameTimeMs, liveBodyCount);
  }
  if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
    renderCameraControls();
  }
  if (ImGui::CollapsingHeader("Spawn", ImGuiTreeNodeFlags_DefaultOpen)) {
    renderSpawnControls();
  }

  ImGui::End();
}

void ImguiController::endFrame() noexcept {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImguiController::renderFrameStats(float frameTimeMs,
                                       size_t liveBodyCount) {
  ImGui::Text("Frame time: %.3f ms (%.1f FPS)", frameTimeMs,
              frameTimeMs > 0.0f ? 1000.0f / frameTimeMs : 0.0f);
  ImGui::Text("Live bodies: %zu", liveBodyCount);
}

void ImguiController::renderCameraControls() {
  int mode = static_cast<int>(cameraMode_);
  ImGui::RadioButton("Orthographic (2D)", &mode,
                     static_cast<int>(CameraMode::Orthographic2D));
  ImGui::RadioButton("Perspective (3D)", &mode,
                     static_cast<int>(CameraMode::Perspective3D));
  cameraMode_ = static_cast<CameraMode>(mode);

  ImGui::Separator();
  if (cameraMode_ == CameraMode::Orthographic2D) {
    ImGui::TextWrapped("Left-drag the world view to pan.");
  } else {
    ImGui::TextWrapped(
        "Left-drag the world view to look around. W/A/S/D to move, "
        "Space/Ctrl for up/down.");
  }
}

void ImguiController::renderSpawnControls() {
  if (ImGui::Button("Spawn Sphere")) { spawnSphereRequested_ = true; }
  ImGui::SameLine();
  if (ImGui::Button("Spawn Box")) { spawnBoxRequested_ = true; }
  if (ImGui::Button("Reset Scene")) { resetRequested_ = true; }
}

bool ImguiController::consumeSpawnSphereRequest() noexcept {
  bool value = spawnSphereRequested_;
  spawnSphereRequested_ = false;
  return value;
}

bool ImguiController::consumeSpawnBoxRequest() noexcept {
  bool value = spawnBoxRequested_;
  spawnBoxRequested_ = false;
  return value;
}

bool ImguiController::consumeResetRequest() noexcept {
  bool value = resetRequested_;
  resetRequested_ = false;
  return value;
}

}  // namespace render
