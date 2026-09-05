#include <imgui-SFML.h>
#include <imgui.h>

#include <algorithm>
#include <limits>
#include <ui/ImguiController.hpp>

namespace {
constexpr float INF = std::numeric_limits<float>::infinity();
}

ImguiController::ImguiController(Simulator& sim)
    : sim_{sim},
      forceType_{ForceType::None},
      forceMagnitude_{2000.0f},
      radialPushRadius_{50.0f},
      particleRadius_{2.0f},
      particleMass_{1.0f},
      spawnType_{SpawnType::None},
      objMult_{1},
      streamSpeed_{1200.0f},
      streamOmega_{0.5f},
      spawnInterval_{0.0167f},
      spawning_{false} {}

void ImguiController::render() {
  ImGui::SetNextWindowSize(ImVec2{450.0f, 0.0f}, ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowPos(ImVec2{0.0f, 0.0f}, ImGuiCond_FirstUseEver);

  ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("Particles: %zu", sim_.particles().size());
  ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

  renderHelp();
  renderParameters();
  renderIntegration();
  renderBroadphase();
  renderSolver();
  renderForces();
  renderSpawn();

  ImGui::End();
}

void ImguiController::renderHelp() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Help")) {
    ImGui::Spacing();
    ImGui::SeparatorText("Forces");
    ImGui::BulletText("Force controls are simple, select a force, and click.");

    ImGui::Spacing();
    ImGui::SeparatorText("Spawning");
    ImGui::BulletText(
        "Manual: Select it and right click on your mouse. You\ncan increment "
        "how many particles are spawned per click.");
    ImGui::BulletText("The rest: Select and press the space bar.");
    ImGui::BulletText(
        "Note that you are activating continuous spawning when\nyou hit the "
        "space bar. So if it's activated and you\nselect another spawning "
        "method, it'll automatically\ndo that one.");
  }
}

void ImguiController::renderParameters() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Parameters")) {
    ImGui::Spacing();
    ImGui::SliderFloat("Gravity", &sim_.gravity, -100.0f, 100.0f);
    ImGui::SliderFloat("Restitution", &sim_.restitution, 0.0f, 1.0f);
  }
}

void ImguiController::renderIntegration() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Integration")) {
    ImGui::Spacing();
    if (ImGui::RadioButton("Euler",
                           sim_.integrationType() == IntegrationType::Euler)) {
      sim_.setIntegrationType(IntegrationType::Euler);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Verlet",
                           sim_.integrationType() == IntegrationType::Verlet)) {
      sim_.setIntegrationType(IntegrationType::Verlet);
    }
  }
}

void ImguiController::renderBroadphase() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Broadphase")) {
    ImGui::Spacing();
    if (ImGui::RadioButton("Naive",
                           sim_.broadphaseType() == BroadphaseType::Naive)) {
      sim_.setBroadphaseType(BroadphaseType::Naive);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Uniform Grid", sim_.broadphaseType() ==
                                               BroadphaseType::SpatialGrid)) {
      sim_.setBroadphaseType(BroadphaseType::SpatialGrid);
    }
  }
}

void ImguiController::renderSolver() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Solver")) {
    ImGui::Spacing();
    int iterations = static_cast<int>(sim_.solverIterations);
    ImGui::SliderInt("Iterations", &iterations, 1, 20);
    sim_.solverIterations = static_cast<size_t>(iterations);
  }
}

void ImguiController::renderForces() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Forces")) {
    ImGui::Spacing();
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
}

void ImguiController::renderSpawn() {
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("Spawn Methods")) {
    ImGui::Spacing();
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

    if (spawnType_ != SpawnType::None) {
      ImGui::InputFloat("Particle Radius", &particleRadius_);
      particleRadius_ = std::clamp(particleRadius_, MIN_PARTICLE_SIZE,
                                   sim_.maxParticleRadius());

      ImGui::InputFloat("Particle Mass", &particleMass_);
      particleMass_ = std::clamp(particleMass_, MIN_PARTICLE_MASS, INF);
    }

    if (spawnType_ == SpawnType::Manual) {
      ImGui::InputInt("Object Multiple", &objMult_, 1, 10);
      objMult_ = std::clamp(objMult_, 1, static_cast<int>(sim_.capacity()));
    }

    if (spawnType_ == SpawnType::Stream) {
      ImGui::InputFloat("Particle Speed", &streamSpeed_, 50.0f, 500.0f);
      streamSpeed_ = std::clamp(streamSpeed_, 1.0f, INF);
      ImGui::InputFloat("Stream Omega\n(angular freq.)", &streamOmega_, 0.1f,
                        1.0f);
    }

    if (spawnType_ == SpawnType::Stream || spawnType_ == SpawnType::Random) {
      ImGui::InputFloat("Spawn Interval", &spawnInterval_, 0.001f, 0.01f,
                        "%.4f");
      spawnInterval_ = std::clamp(spawnInterval_, 0.001f, 1.0f);

      const float rate = 1.0f / spawnInterval_;
      ImGui::SameLine();
      ImGui::Text("~ %.0f particles/s", rate);
    }
  }
}
