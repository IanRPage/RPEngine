#include <sim/Simulator.hpp>

Simulator::Simulator(SimConfig config) noexcept : config_(config) {}

void Simulator::advance(float realDeltaTime) noexcept {
  accumulator_ += realDeltaTime;
  int steps = 0;
  bool steppedAtLeastOnce = false;

  while (accumulator_ >= config_.fixedDt && steps < config_.maxStepsPerFrame) {
    if (!steppedAtLeastOnce) {
      world_.snapshotPrevState();
      steppedAtLeastOnce = true;
    }
    world_.step(config_.fixedDt);
    accumulator_ -= config_.fixedDt;
    steps++;
  }

  alpha_ = accumulator_ / config_.fixedDt;
  lastStepsTaken_ = steps;
}
