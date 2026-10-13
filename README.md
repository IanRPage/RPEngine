# RPEngine

This is my attempt at building a rigid-body physics engine. I made this to
practice implementing physics in code and to practice algorithms. But mainly
this just felt like a fun project to make.

## Project Structure

The project is split into four top-level modules:

- `rpe-physics`: rigid-body simulation, collision detection, and broadphase
- `rpe-render`: reusable OpenGL window, camera, shader, mesh, and batching code
- `rpengine`: simulation loop and application-specific physics/render glue
- `tests`: unit tests linked against the module libraries

## Examples

![RPEngine demo scene](images/demo-perspective3d.png?raw=true)

## Building

**Prerequisites:**

- C++20 compiler, CMake 3.28+, and `git`
- `pkg-config`
- Python 3 with `jinja2` installed (`python3 -c "import jinja2"` to check)

### Debian/Ubuntu:

```
sudo apt update
sudo apt install build-essential cmake git python3 python3-jinja2 pkg-config \
    libwayland-dev libxkbcommon-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev \
    libgl1-mesa-dev
```

### Fedora:

```
sudo dnf install gcc-c++ cmake git python3 python3-jinja2 pkgconf-pkg-config \
    wayland-devel libxkbcommon-devel \
    libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel libXext-devel \
    mesa-libGL-devel
```

### Arch:

```
sudo pacman -S base-devel cmake git python python-jinja pkgconf \
    wayland libxkbcommon \
    libx11 libxrandr libxinerama libxcursor libxi libxext \
    mesa
```

After dependencies are installed, configure and build normally:

```
cmake -B build -S . -DCMAKE_BUILD_TYPE=Debug    # or Release
cmake --build build -j
```

After the initial configure step above, these targets reconfigure and build:


```
cmake --build build --target debug    # configures Debug, then builds
cmake --build build --target release  # configures Release, then builds
```

The libraries can also be configured and built independently:

```sh
cmake -S rpe-physics -B build-physics
cmake --build build-physics -j

cmake -S rpe-render -B build-render
cmake --build build-render -j
```

**FOR WINDOWS USERS**: Untested since the refactor. GLFW itself supports Windows
fine; you're on your own for the exact dependency setup. Feel free to find out
and open a PR making changes to these instructions.

**FOR MACOS USERS**: The renderer targets OpenGL 4.1 core instead of a newer
version because Apple's OpenGL implementation has been frozen at 4.1. That said,
it hasn't been built/tested on macOS yet since the rewrite. If you're on macOS,
feel free to find out and open a PR making changes to these instructions.

## Testing

Unit tests live in `tests/` and link the module libraries directly. They cover
math, shapes/collision, broadphase, narrowphase/GJK/EPA, solver/dynamics, the
fixed timestep, and engine renderable state with GoogleTest.

Configure and build the test binary:

```
cmake -B build -S .
cmake --build build --target rp_tests -j
```

Then run the tests, either directly:

```
./build/bin/rp_tests
```

or using ctest for per-case pass/fail output:

```
ctest --test-dir build --output-on-failure
```

Tests are built by default. To skip them (avoid GoogleTest fetch/build),
configure with `-DBUILD_TESTS=OFF`.

## Tuning the Simulation

```cpp
Simulator sim;                              // SimConfig{fixedDt = 1/60, maxStepsPerFrame = 5}
World& world = sim.world();                 // world.gravity() defaults to (0, -9.81, 0)

world.createStaticBody(
    BoxShape{Vec3f(13.5f, 0.5f, 13.5f)},    // half-extents
    Transform{Vec3f(0, -0.5f, 0)},          // position, identity orientation
    /*friction=*/0.5f,
    /*restitution=*/0.3f);

Transform transform{Vec3f(0, 1, 0)};

world.createDynamicBody(
    SphereShape{0.5f}, transform,
    /*mass=*/1.0f,
    /*friction=*/0.5f,
    /*restitution=*/0.5f,
    /*constrainTo2D=*/false);
```

## Controls

The sidebar panel (`ImguiController`) is organized by section, and this is
everything it currently exposes:

**Camera**:
- **Orthographic (2D)**: Left-drag the world view to pan. No zoom control
  yet.
- **Perspective (3D)**: Click the world view to capture the mouse, then look
  around with the mouse and move with W/A/S/D (Q/E for up/down), Esc releases
  the cursor.

**Spawn**: "Spawn Sphere"/"Spawn Box"/"Spawn Capsule" each drop one body of
that shape at a randomized position above the scene. "Reset Scene" removes every
dynamic body, leaving statics in place.

**Screenshot**: "Take Screenshot" opens a path prompt and writes the current
framebuffer to a PNG (this is how the screenshot above was captured).

## State of Simulation Performance

These results from [`rp_bench`](benchmarks/README.md) are as of 10/12/26 (commit
`33c070a`), running single threaded, Release mode, with a i7-13800H (pinned to a
P-core, turbo on). Each case settles for 300 steps, then times 300 steps at `dt
= 1/60`, 5 repetitions (all CVs < 0.6%).

| Case | Per step | Per body-step | 60 Hz budget (16.7 ms) |
|---|---|---|---|
| `BoxStack/h:5` | 0.031 ms | ~6 µs | 0.2% |
| `MixedShapes/n:1000` | 8.1 ms | 8.1 µs | 49% |
| `SpherePile/n:10000` | 46.9 ms | 4.7 µs | 2.8x over |
| `MixedShapes/n:10000` | 103 ms | 10.3 µs | 6.2x over |

So ~1k mixed bodies run in real time, 10k don't, and the 100k-body target is at
least ~60x away. 1k to 10k scales slightly worse than linear (12.7x for 10x the
bodies), and boxes/capsules cost ~2.2x more than spheres (likely GJK/EPA vs. the
sphere-sphere fast path).

## TODO

- [ ] add multithreading (broadphase makes a flat pair/manifold list per fixed
  step, can parallelize here)
- [ ] stabilize FPS reading to make it actually readable
- [ ] fix combination of Q/E up/down movement when looking around
- [ ] add a "Take Screen Recording" button
- [ ] add zoom control to orthographic camera
- [ ] implement hot-reloading for quicker debugging
- [ ] add some kind of profiler that runs a simulation without UI
- [ ] wire `World::addWorldBoundaries` into `src/main.cpp`'s demo scene, so
  shipped demo actually shows a bounded volume
- [ ] add ImGui controls for gravity and restitution
- [ ] add broadphase comparison toggle (Naive/Grid/DynamicBVH) to the panel.
  `World::setBroadphase` can swap it at runtime, just needs the UI
- [ ] add a narrowphase comparison toggle 
  - [ ] add more narrowphase algorithms
- [ ] add manual click-to-spawn in the 2D orthographic view 
- [ ] update README with an current demo gif
- [ ] make piles actually settle (likely needs a restitution velocity threshold)
- [x] re-benchmark simulation performance
- [x] refactor
  - [x] turn physics code into a separate library
  - [x] turn rendering code into a separate library
  - [x] rebuild RPEngine using this library approach
