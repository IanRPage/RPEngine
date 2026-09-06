#ifndef RPENGINE_SIM_SIMCONFIG_HPP
#define RPENGINE_SIM_SIMCONFIG_HPP

struct SimConfig {
  float fixedDt = 1.0f / 60.0f;
  int maxStepsPerFrame = 5;  // spiral-of-death guard
};

#endif
