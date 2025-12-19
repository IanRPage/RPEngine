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
        spawnType_{SpawnType::None},
        objMult_{1} {}

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
      particleRadius_ = std::clamp(particleRadius_, MIN_PARTICLE_SIZE,
                                   sim_.maxParticleRadius());
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

    ImGui::Spacing();
    ImGui::SeparatorText("Forces");
    {
      ImGui::Text(
          "Select one of the below forces and then left-click to\napply it.");
      ImGui::Spacing();

      int curr = toInt(sim_.forceType());

      if (ImGui::RadioButton("None", &curr, toInt(ForceType::None))) {
      }
      ImGui::SameLine();
      if (ImGui::RadioButton("Radial", &curr, toInt(ForceType::Radial))) {
      }

      sim_.setForceType(fromInt<ForceType>(curr));

      if (sim_.forceType() == ForceType::Radial) {
        ImGui::InputFloat("Radial Push Radius", &radialPushRadius_);
        radialPushRadius_ =
            std::clamp(radialPushRadius_, MIN_PARTICLE_SIZE, 1e7f);
      }

      if (sim_.forceType() != ForceType::None) {
        ImGui::InputFloat("Magnitude", &forceMagnitude_);
      }
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Spawn Methods");
    {
      ImGui::Text(
          "Select one of the below spawning methods and then press\n<Space> to "
          "toggle it on or off.");
      ImGui::Spacing();

      int curr = toInt(spawnType_);

      if (ImGui::RadioButton("None", &curr, toInt(SpawnType::None))) {
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Random", &curr, toInt(SpawnType::Random))) {
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Stream", &curr, toInt(SpawnType::Stream))) {
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Max", &curr, toInt(SpawnType::Max))) {
      }

      spawnType_ = fromInt<SpawnType>(curr);

      if (spawnType_ == SpawnType::Random) {
        ImGui::InputInt("Object Multiple", &objMult_);
      }
    }

    ImGui::End();
  }

  float particleRadius() const noexcept { return particleRadius_; }
  float radialPushRadius() const noexcept { return radialPushRadius_; }
  // ForceType forceType() const noexcept { return forceType_; }
  float forceMagnitude() const noexcept { return forceMagnitude_; }
  SpawnType spawnType() const noexcept { return spawnType_; }
  int objMultiple() const noexcept { return objMult_; }

 private:
  Simulator& sim_;
  float particleRadius_;
  float radialPushRadius_;
  // ForceType forceType_;
  float forceMagnitude_;
  SpawnType spawnType_;
  int objMult_;

  template <typename E>
  static constexpr int toInt(E e) {
    return static_cast<int>(e);
  }

  template <typename E>
  static constexpr E fromInt(int i) {
    return static_cast<E>(i);
  }
};

#endif
