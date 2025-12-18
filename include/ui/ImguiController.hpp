#ifndef IMGUI_CONTROLLER_HPP
#define IMGUI_CONTROLLER_HPP

#include <imgui-SFML.h>
#include <imgui.h>

#include <Simulator.hpp>
#include <algorithm>

class ImguiController {
 public:
  ImguiController(Simulator& sim, float particleRadius = 2.0f)
      : sim_{sim},
        particleRadius_{particleRadius},
        radialPushRadius_{50.0f},
        forceMagnitude_{2000.0f},
        spawnType_{SpawnType::None} {}

  void render() {
    ImGui::SetNextWindowSize(ImVec2{450.0f, 500.0f});
    ImGui::SetNextWindowPos(ImVec2{0.0f, 0.0f});
    ImGui::Begin("Simulation");

    ImGui::Text("Particles: %zu", sim_.particles().size());
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::Spacing();
    ImGui::SeparatorText("Parameters");
    {
      ImGui::InputFloat("Particle Radius", &particleRadius_);
      auto temp = std::clamp(particleRadius_, MIN_PARTICLE_SIZE,
                             sim_.maxParticleRadius());
      particleRadius_ = temp;
      ImGui::SliderFloat("Gravity", &sim_.gravity, -100.0f, 100.0f);
      ImGui::SliderFloat("Restitution", &sim_.restitution, 0.0f, 1.0f);
      ImGui::InputFloat("Radial Push Radius", &radialPushRadius_);
      radialPushRadius_ =
          std::clamp(radialPushRadius_, MIN_PARTICLE_SIZE, 1e7f);
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

    ImGui::Spacing();
    ImGui::SeparatorText("Forces");
    {
      ImGui::Text(
          "Select one of the below forces and then left-click to\napply it.");
      ImGui::Spacing();

      if (ImGui::RadioButton("None", sim_.forceType() == ForceType::None)) {
        sim_.setForceType(ForceType::None);
      }
      ImGui::SameLine();
      if (ImGui::RadioButton("Radial", sim_.forceType() == ForceType::Radial)) {
        sim_.setForceType(ForceType::Radial);
      }
      ImGui::InputFloat("Magnitude", &forceMagnitude_);
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Spawn Methods");
    {
      ImGui::Text(
          "Select one of the below spawning methods and then press\n<Space> to "
          "toggle it on or off.");
      ImGui::Spacing();

      if (ImGui::RadioButton("None", spawnType_ == SpawnType::None)) {
        spawnType_ = SpawnType::None;
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Random", spawnType_ == SpawnType::Random)) {
        spawnType_ = SpawnType::Random;
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Stream", spawnType_ == SpawnType::Stream)) {
        spawnType_ = SpawnType::Stream;
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Max", spawnType_ == SpawnType::Max)) {
        spawnType_ = SpawnType::Max;
      }
    }

    ImGui::End();
  }

  float particleRadius() const noexcept { return particleRadius_; }
  float radialPushRadius() const noexcept { return radialPushRadius_; }
  float forceMagnitude() const noexcept { return forceMagnitude_; }
  SpawnType spawnType() const noexcept { return spawnType_; }

 private:
  Simulator& sim_;
  float particleRadius_;
  float radialPushRadius_;
  float forceMagnitude_;
  SpawnType spawnType_;
};

#endif
