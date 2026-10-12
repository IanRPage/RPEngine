#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <random>
#include <string>

#include <core/World.hpp>

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr int kSteps = 600;

void buildBoxStack(World& world, int height) {
  Transform groundT;
  groundT.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(50.0f, 0.5f, 50.0f)}},
                         groundT, /*friction=*/0.5f, /*restitution=*/0.0f);
  for (int i = 0; i < height; i++) {
    Transform t;
    t.position = Vec3f(0.0f, 0.5f + static_cast<float>(i) * 1.01f, 0.0f);
    world.createDynamicBody(ShapeVariant{BoxShape{Vec3f(0.5f)}}, t,
                            /*mass=*/1.0f, /*friction=*/0.5f,
                            /*restitution=*/0.0f);
  }
}

enum class PileKind { Spheres, Mixed };

void buildPile(World& world, PileKind kind, int n) {
  const float cell = kind == PileKind::Spheres ? 1.2f : 1.5f;
  const int layers =
      std::max(1, static_cast<int>(std::lround(std::cbrt(n) / 2.0)));
  const int k = static_cast<int>(std::ceil(std::sqrt(double(n) / layers)));
  const int usedLayers = (n + k * k - 1) / (k * k);
  const float halfWidth = 0.5f * (static_cast<float>(k) * cell + 0.5f);
  const float height = static_cast<float>(usedLayers) * cell + 1.5f;
  world.addWorldBoundaries(Vec3f(-halfWidth, 0.0f, -halfWidth),
                           Vec3f(halfWidth, height, halfWidth),
                           /*thickness=*/1.0f, /*friction=*/0.5f,
                           /*restitution=*/0.2f, /*is2D=*/false);

  std::mt19937 rng(0x5EED);
  std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
  std::normal_distribution<float> gauss;
  for (int i = 0; i < n; i++) {
    const int layer = i / (k * k);
    const int ix = (i % (k * k)) % k;
    const int iz = (i % (k * k)) / k;
    Transform t;
    t.position =
        Vec3f(-halfWidth + 0.25f + cell * (static_cast<float>(ix) + 0.5f) +
                  jitter(rng),
              0.25f + cell * (static_cast<float>(layer) + 0.5f),
              -halfWidth + 0.25f + cell * (static_cast<float>(iz) + 0.5f) +
                  jitter(rng));

    ShapeVariant shape{SphereShape{0.5f}};
    if (kind == PileKind::Mixed) {
      t.orientation =
          glm::normalize(Quatf(gauss(rng), gauss(rng), gauss(rng), gauss(rng)));
      if (i % 3 == 0) { shape = ShapeVariant{BoxShape{Vec3f(0.4f)}}; }
      if (i % 3 == 2) { shape = ShapeVariant{CapsuleShape{0.3f, 0.4f}}; }
    }
    world.createDynamicBody(shape, t, /*mass=*/1.0f, /*friction=*/0.5f,
                            /*restitution=*/0.2f);
  }
}

template <class Build>
void runScene(benchmark::State& state, const Build& build) {
  double steppingSeconds = 0.0;
  int64_t steps = 0;
  size_t bodies = 0;

  for (auto _ : state) {
    World world;
    world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));
    build(world);
    bodies = world.bodies().size();

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kSteps; i++) { world.step(kDt); }
    const auto t1 = std::chrono::steady_clock::now();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    state.SetIterationTime(seconds);
    steppingSeconds += seconds;
    steps += kSteps;
  }

  state.counters["bodies"] = static_cast<double>(bodies);
  state.counters["ms_per_step"] =
      1e3 * steppingSeconds / static_cast<double>(std::max<int64_t>(1, steps));
}

template <class Build>
void add(const std::string& name, Build build) {
  benchmark::RegisterBenchmark(
      name, [build](benchmark::State& state) { runScene(state, build); })
      ->UseManualTime()
      ->Unit(benchmark::kMillisecond);
}

void addPile(PileKind kind, int n) {
  const char* prefix = kind == PileKind::Spheres ? "SpherePile" : "MixedShapes";
  add(std::string(prefix) + "/n:" + std::to_string(n),
      [kind, n](World& w) { buildPile(w, kind, n); });
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

  add("BoxStack/h:20", [](World& w) { buildBoxStack(w, 20); });
  addPile(PileKind::Mixed, 1000);
  addPile(PileKind::Mixed, 10000);
  addPile(PileKind::Spheres, 10000);
  if (large) {
    addPile(PileKind::Mixed, 100000);
    addPile(PileKind::Spheres, 100000);
  }

  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) { return 1; }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
