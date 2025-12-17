#include <imgui-SFML.h>
#include <imgui.h>

#include <Simulator.hpp>

class ImguiController {
 public:
  ImguiController(Simulator& sim) : sim_{sim} {}

  void render() {
    ImGui::SetNextWindowSize(ImVec2{400.0f, 300.0f});
    ImGui::SetNextWindowPos(ImVec2{10.0f, 10.0f});
    ImGui::Begin("Simulation");

    ImGui::Text("Particles: %zu", sim_.particles().size());
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::SeparatorText("Parameters");
    {
      ImGui::SliderFloat("Gravity", &sim_.gravity, -100.0f, 100.0f);
      ImGui::SliderFloat("Restitution", &sim_.restitution, 0.0f, 1.0f);
    }

    ImGui::SeparatorText("Integration");
    {
      if (ImGui::RadioButton(
              "Euler", sim_.integrationType() == IntegrationType::Euler)) {
        sim_.setIntegrationType(IntegrationType::Euler);
      }
      ImGui::SameLine();
      if (ImGui::RadioButton(
              "Verlet", sim_.integrationType() == IntegrationType::Verlet)) {
        sim_.setIntegrationType(IntegrationType::Verlet);
      }
    }

    ImGui::SeparatorText("Broadphase");
    {
      if (ImGui::RadioButton("Naive",
                             sim_.broadphaseType() == BroadphaseType::Naive)) {
        sim_.setBroadphaseType(BroadphaseType::Naive);
      }
      ImGui::SameLine();
      if (ImGui::RadioButton("Quad Tree",
                             sim_.broadphaseType() == BroadphaseType::Qtree)) {
        sim_.setBroadphaseType(BroadphaseType::Qtree);
      }
      ImGui::SameLine();
      if (ImGui::RadioButton("Uniform Grid", sim_.broadphaseType() ==
                                                 BroadphaseType::UniformGrid)) {
        sim_.setBroadphaseType(BroadphaseType::UniformGrid);
      }
    }

    ImGui::End();
  }

 private:
  Simulator& sim_;
};
