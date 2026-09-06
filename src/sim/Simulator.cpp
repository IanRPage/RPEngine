#include <sim/Simulator.hpp>

Simulator::Simulator(SimConfig config) noexcept : config_(config) {}

void Simulator::advance(float realDeltaTime) noexcept {
  accumulator_ += realDeltaTime;
  int steps = 0;
  bool steppedAtLeastOnce = false;

  while (accumulator_ >= config_.fixedDt && steps < config_.maxStepsPerFrame) {
    if (!steppedAtLeastOnce) {
      // Snapshot BEFORE the first substep of this advance() call, so the
      // interpolation window spans the whole gap since the last render
      // frame, not just the last substep taken this call.
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
