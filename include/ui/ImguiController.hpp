#ifndef IMGUI_CONTROLLER_HPP
#define IMGUI_CONTROLLER_HPP

#include <imgui-SFML.h>
#include <imgui.h>

#include <Simulator.hpp>
#include <algorithm>

constexpr float MIN_PARTICLE_SIZE = 0.5f;

class ImguiController {
 public:
  ImguiController(Simulator& sim, float particleRadius = 2.0f)
      : sim_{sim}, particleRadius_{particleRadius} {}

  void render() {
    ImGui::SetNextWindowSize(ImVec2{400.0f, 300.0f});
    ImGui::SetNextWindowPos(ImVec2{10.0f, 10.0f});
    ImGui::Begin("Simulation");

    ImGui::Text("Particles: %zu", sim_.particles().size());
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::Spacing();
    ImGui::SeparatorText("Parameters");
    {
      ImGui::InputFloat("Particle Radius", &particleRadius_);
      auto t = std::clamp(particleRadius_, MIN_PARTICLE_SIZE,
                          sim_.maxParticleRadius());
      particleRadius_ = t;
      ImGui::SliderFloat("Gravity", &sim_.gravity, -100.0f, 100.0f);
      ImGui::SliderFloat("Restitution", &sim_.restitution, 0.0f, 1.0f);
    }

    ImGui::Spacing();
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

    ImGui::Spacing();
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
                                                 BroadphaseType::SpatialGrid)) {
        sim_.setBroadphaseType(BroadphaseType::SpatialGrid);
      }
    }

    ImGui::End();
  }

  float particleRadius() const noexcept { return particleRadius_; }

 private:
  Simulator& sim_;
  float particleRadius_;
};

#endif
