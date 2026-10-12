# RPEngine headless benchmarks (`rp_bench`)

Timing-only performance suite for `rpe-physics`, built on
[Google Benchmark](https://github.com/google/benchmark). Links **only**
`rpe-physics` (no GLFW, OpenGL or ImGui). Everything is in
[`bench_physics.cpp`](bench_physics.cpp).

The physics correctness checks for the same scenes (bounce matches restitution,
stacks settle, piles stay inside their walls) are regular tests in
[`tests/test_scenarios.cpp`](../tests/test_scenarios.cpp).

## Build

From the root project:

```bash
cmake -S . -B build -DBUILD_BENCHMARKS=ON   # Release is the default
cmake --build build --target rp_bench
```

Or standalone (fetches only GLM + Google Benchmark):

```bash
cmake -S benchmarks -B build/bench
cmake --build build/bench
```

Timings are only meaningful in `Release` / `RelWithDebInfo`.

## Run

```bash
build/bin/rp_bench                                   # default suite
build/bin/rp_bench --benchmark_filter='MixedShapes'  # one scene
build/bin/rp_bench --rpe_large                       # + 100k-body cases
build/bin/rp_bench --benchmark_out=results.json --benchmark_out_format=json
```

Compare two runs (e.g. before/after an engine change):

```bash
python3 build/_deps/googlebenchmark-src/tools/compare.py benchmarks before.json after.json
```

> For numbers you'll compare across runs, set the CPU governor to
> `performance` and pass `--benchmark_repetitions=5`.

## Cases

| Case | Scene | Stresses |
|---|---|---|
| `BoxStack/h:20` | 20 unit boxes stacked on a ground box | solver (contact-heavy, few bodies) |
| `MixedShapes/n:{1000,10000}` | boxes / spheres / capsules (random orientations) on a jittered grid inside `addWorldBoundaries` walls | everything, incl. GJK/EPA |
| `SpherePile/n:10000` | same container, spheres only | broadphase + solver without GJK/EPA box/capsule cost |
| `--rpe_large`: `{MixedShapes,SpherePile}/n:100000` | as above | scaling |

All scenes use the default `World` (BVH broadphase, default `SolverConfig`),
`dt = 1/60`, and are deterministic for a given standard library.

## What's measured

One iteration = build the scene, then 600 × `world.step(1/60)` (10 s
simulated). **Only the stepping is timed** (manual timing around the step
loop), so setup and teardown are excluded and `rpe-physics` needs no
instrumentation.

- `Time`: one 600-step run
- `ms_per_step`: mean wall-clock time of one `World::step`
- `bodies`: total bodies, including static ground/walls
