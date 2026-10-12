# RPEngine headless benchmarks (`rp_bench`)

This is the performance suite for `rpe-physics` that's built with
[Google Benchmark](https://github.com/google/benchmark).

## Build

Do the following to build the project with the benchmarking suite (from root):

```bash
cmake -S . -B build -DBUILD_BENCHMARKS=ON
cmake --build build --target rp_bench
```

Could also just build the standalone benchmark without RPEngine itself:

```bash
cmake -S benchmarks -B build/bench
cmake --build build/bench
```

> It's pretty obvious but the results of the benchmark are only meaningful in
> `Release` mode.

## Run

Use the `rp_bench` CLI tool to run the benchmark.

```bash
build/bin/rp_bench                                   # default suite
build/bin/rp_bench --benchmark_filter='MixedShapes'  # specific scene
build/bin/rp_bench --rpe_large                       # 100k-body big dawg
build/bin/rp_bench --benchmark_out=results.json --benchmark_out_format=json
```

Then to compare two runs, say, before and after a change to the engine, do this:

```bash
python3 build/_deps/googlebenchmark-src/tools/compare.py benchmarks before.json after.json
```

> For numbers you'll compare across runs, I recommend setting the CPU governor
> to `performance` and pass `--benchmark_repetitions=5`.

```bash
sudo cpupower frequency-set -g performance   # restore this after
lscpu -e=CPU,MAXMHZ                          # pick a P-core if u have a hybrid cpu (higher MAXMHZ)
taskset -c 2 build/bin/rp_bench --benchmark_repetitions=5 \
  --benchmark_report_aggregates_only=true --benchmark_out=before.json
```

Rerun if any `_cv` row is above ~3%. At least for laptops, disabling turbo
(`echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo`) trades
speed for repeatability. Either way, use the same setup for both runs. Five
repetitions take ~10-15 min (`--rpe_large` much longer).

## Cases

| Case | Scene | Stresses |
|---|---|---|
| `BoxStack/h:20` | 20 unit boxes stacked on a ground box | solver (contact-heavy, few bodies) |
| `MixedShapes/n:{1000,10000}` | boxes / spheres / capsules (random orientations) on a jittered grid inside `addWorldBoundaries` walls | everything, incl. GJK/EPA |
| `SpherePile/n:10000` | same container, spheres only | broadphase + solver without GJK/EPA box/capsule cost |
| `--rpe_large`: `{MixedShapes,SpherePile}/n:100000` | as above | scaling |

## Metrics

One iteration = build scene, 600 × `world.step(1/60)`. Only stepping is
timed, meaning setup and teardown are excluded. It measures

- `Time`: one 600-step run
- `ms_per_step`: mean wall-clock time of one `World::step`
- `bodies`: total bodies, including static ground/walls
