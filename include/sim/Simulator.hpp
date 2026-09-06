#ifndef RPENGINE_SIM_SIMULATOR_HPP
#define RPENGINE_SIM_SIMULATOR_HPP

#include <core/World.hpp>
#include <sim/SimConfig.hpp>

class Simulator {
 public:
  explicit Simulator(SimConfig config = {}) noexcept;

  void advance(float realDeltaTime) noexcept;

  float interpolationAlpha() const noexcept { return alpha_; }
  int stepsTakenLastAdvance() const noexcept { return lastStepsTaken_; }

  World& world() noexcept { return world_; }
  const World& world() const noexcept { return world_; }
  const SimConfig& config() const noexcept { return config_; }

 private:
  SimConfig config_;
  World world_;
  float accumulator_ = 0.0f;
  float alpha_ = 0.0f;
  int lastStepsTaken_ = 0;
};

#endif
