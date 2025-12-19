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
        spawning_{false},
        objMult_{1} {}

  void render() {
    ImGui::SetNextWindowSize(ImVec2{450.0f, 0.0f}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2{0.0f, 0.0f}, ImGuiCond_FirstUseEver);

    ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Particles: %zu", sim_.particles().size());
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    // ------ Parameters ------
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Parameters")) {
      ImGui::InputFloat("Particle Radius", &particleRadius_);
      particleRadius_ = std::clamp(particleRadius_, MIN_PARTICLE_SIZE,
                                   sim_.maxParticleRadius());
      ImGui::SliderFloat("Gravity", &sim_.gravity, -100.0f, 100.0f);
      ImGui::SliderFloat("Restitution", &sim_.restitution, 0.0f, 1.0f);
    }

    // ------ Integration ------
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Integration")) {
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

    // ------ Broadphase ------
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Broadphase")) {
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

    // ------ Forces ------
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Forces")) {
      ImGui::Text(
          "Select one of the below forces and then left-click to\napply it.");
      ImGui::Spacing();

      int curr = toInt(forceType_);

      if (ImGui::RadioButton("None", &curr, toInt(ForceType::None))) {
      }
      ImGui::SameLine();
      if (ImGui::RadioButton("Radial", &curr, toInt(ForceType::Radial))) {
      }

      forceType_ = fromInt<ForceType>(curr);

      if (forceType_ == ForceType::Radial) {
        ImGui::InputFloat("Radial Push Radius", &radialPushRadius_);
        radialPushRadius_ =
            std::clamp(radialPushRadius_, MIN_PARTICLE_SIZE, 1e7f);
      }

      if (forceType_ != ForceType::None) {
        ImGui::InputFloat("Magnitude", &forceMagnitude_);
      }
    }

    // ------ Spawning Methods ------
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Spawn Methods")) {
      ImGui::Text(
          "Select one of the below spawning methods and then press\n<Space> to "
          "toggle it on or off.");
      ImGui::Spacing();

      int curr = toInt(spawnType_);

      if (ImGui::RadioButton("None", &curr, toInt(SpawnType::None))) {
      }
      ImGui::SameLine();

      if (ImGui::RadioButton("Manual", &curr, toInt(SpawnType::Manual))) {
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

      if (spawnType_ == SpawnType::Random || spawnType_ == SpawnType::Manual) {
        ImGui::InputInt("Object Multiple", &objMult_);
      }
    }

    ImGui::End();
  }

  float particleRadius() const noexcept { return particleRadius_; }
  float radialPushRadius() const noexcept { return radialPushRadius_; }
  ForceType forceType() const noexcept { return forceType_; }
  float forceMagnitude() const noexcept { return forceMagnitude_; }
  SpawnType spawnType() const noexcept { return spawnType_; }
  int objMultiple() const noexcept { return objMult_; }
  bool spawning() const noexcept { return spawning_; }
  void setSpawning(bool flag) noexcept { spawning_ = flag; }

 private:
  Simulator& sim_;
  float particleRadius_;
  float radialPushRadius_;
  ForceType forceType_;
  float forceMagnitude_;
  SpawnType spawnType_;
  bool spawning_;
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
