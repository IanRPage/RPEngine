#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <optional>
#include <string>

#include "scenes.hpp"

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr int kSettleSteps = 300;
constexpr int kTimedSteps = 300;

template <class Build>
void runScene(benchmark::State& state, const Build& build) {
  size_t bodies = 0;

  for (auto _ : state) {
    state.PauseTiming();
    std::optional<World> world(std::in_place);
    const scenes::Scene scene = build(*world);
    bodies = world->bodies().size();
    for (int i = 0; i < kSettleSteps; i++) { world->step(kDt); }
    state.ResumeTiming();

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kTimedSteps; i++) { world->step(kDt); }
    const auto t1 = std::chrono::steady_clock::now();

    state.PauseTiming();
    const bool intact = std::ranges::all_of(scene.bodies, [&](BodyHandle h) {
      return scene.contains(world->bodies().position(h));
    });
    world.reset();
    state.SetIterationTime(std::chrono::duration<double>(t1 - t0).count());
    state.ResumeTiming();
    if (!intact) {
      state.SkipWithError("a body left the scene (toppled, escaped or NaN)");
      break;
    }
  }

  state.counters["bodies"] = static_cast<double>(bodies);
}

template <class Build>
void add(const std::string& name, Build build) {
  benchmark::RegisterBenchmark(
      name, [build](benchmark::State& state) { runScene(state, build); })
      ->UseManualTime()
      ->Unit(benchmark::kMillisecond);
}

void addPile(bool mixed, int n) {
  add(std::string(mixed ? "MixedShapes" : "SpherePile") +
          "/n:" + std::to_string(n),
      [mixed, n](World& w) { return scenes::buildPile(w, mixed, n); });
}

}  // namespace

int main(int argc, char** argv) {
  bool large = false;
  int out = 1;
  for (int in = 1; in < argc; in++) {
    if (std::strcmp(argv[in], "--rpe_large") == 0) {
      large = true;
    } else {
      argv[out++] = argv[in];
    }
  }
  argc = out;

  add("BoxStack/h:5", [](World& w) { return scenes::buildBoxStack(w, 5); });
  addPile(/*mixed=*/true, 1000);
  addPile(/*mixed=*/true, 10000);
  addPile(/*mixed=*/false, 10000);
  if (large) {
    addPile(/*mixed=*/true, 100000);
    addPile(/*mixed=*/false, 100000);
  }

  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) { return 1; }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
