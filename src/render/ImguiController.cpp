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
  if (ImGui::CollapsingHeader("Screenshot", ImGuiTreeNodeFlags_DefaultOpen)) {
    renderScreenshotControls();
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
        "Click the world view to capture the mouse: look around, W/A/S/D "
        "to move, Space/Ctrl for up/down, all together. Esc releases the "
        "cursor.");
  }
}

void ImguiController::renderSpawnControls() {
  if (ImGui::Button("Spawn Sphere")) { spawnSphereRequested_ = true; }
  ImGui::SameLine();
  if (ImGui::Button("Spawn Box")) { spawnBoxRequested_ = true; }
  if (ImGui::Button("Spawn Capsule")) { spawnCapsuleRequested_ = true; }
  if (ImGui::Button("Reset Scene")) { resetRequested_ = true; }
}

void ImguiController::renderScreenshotControls() {
  if (ImGui::Button("Take Screenshot")) {
    ImGui::OpenPopup("Save Screenshot");
  }

  const ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos(
      ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
      ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (ImGui::BeginPopupModal("Save Screenshot", nullptr,
                            ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Save screenshot to:");
    ImGui::InputText("##ScreenshotPath", screenshotPathBuffer_,
                     sizeof(screenshotPathBuffer_));
    ImGui::TextDisabled("Relative paths are relative to the working "
                        "directory RPEngine was launched from.");
    ImGui::Separator();

    if (ImGui::Button("Save", ImVec2(120, 0))) {
      screenshotPath_ = screenshotPathBuffer_;
      screenshotRequested_ = true;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SetItemDefaultFocus();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
    ImGui::EndPopup();
  }

  if (!lastScreenshotStatus_.empty()) {
    ImGui::TextWrapped("%s", lastScreenshotStatus_.c_str());
  }
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

bool ImguiController::consumeSpawnCapsuleRequest() noexcept {
  bool value = spawnCapsuleRequested_;
  spawnCapsuleRequested_ = false;
  return value;
}

bool ImguiController::consumeResetRequest() noexcept {
  bool value = resetRequested_;
  resetRequested_ = false;
  return value;
}

bool ImguiController::consumeScreenshotRequest(std::string& path) {
  if (!screenshotRequested_) { return false; }
  screenshotRequested_ = false;
  path = screenshotPath_;
  return true;
}

}  // namespace render
