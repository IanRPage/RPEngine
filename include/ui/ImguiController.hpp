#ifndef IMGUI_CONTROLLER_HPP
#define IMGUI_CONTROLLER_HPP

#include <Simulator.hpp>

class ImguiController {
 public:
  ImguiController(Simulator& sim);

  void render();

  // ------ setters ------
  void setSpawning(bool flag) noexcept { spawning_ = flag; }

  // ------ getters ------
  ForceType forceType() const noexcept { return forceType_; }
  float forceMagnitude() const noexcept { return forceMagnitude_; }
  float radialPushRadius() const noexcept { return radialPushRadius_; }
  float particleRadius() const noexcept { return particleRadius_; }
  float particleMass() const noexcept { return particleMass_; }
  SpawnType spawnType() const noexcept { return spawnType_; }
  int objMultiple() const noexcept { return objMult_; }
  float streamSpeed() const noexcept { return streamSpeed_; }
  float streamOmega() const noexcept { return streamOmega_; }
  float spawnInterval() const noexcept { return spawnInterval_; }
  bool spawning() const noexcept { return spawning_; }

 private:
  Simulator& sim_;
  ForceType forceType_;
  float forceMagnitude_;
  float radialPushRadius_;
  float particleRadius_;
  float particleMass_;
  SpawnType spawnType_;
  int objMult_;
  float streamSpeed_;
  float streamOmega_;
  float spawnInterval_;
  bool spawning_;

  template <typename E>
  static constexpr int toInt(E e) {
    return static_cast<int>(e);
  }

  template <typename E>
  static constexpr E fromInt(int i) {
    return static_cast<E>(i);
  }

  void renderHelp();
  void renderParameters();
  void renderIntegration();
  void renderBroadphase();
  void renderSolver();
  void renderForces();
  void renderSpawn();
};

#endif
