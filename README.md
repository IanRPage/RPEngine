# RPEngine

This is my attempt at building a rigid-body physics engine. I made this to
practice implementing physics in code and to practice algorithms. But mainly
this just felt like a fun project to make.

## Examples

![RPEngine demo scene](images/demo-perspective3d.png?raw=true)

## Building

**Prerequisites:**

- A C++20 compiler, CMake 3.28+, and `git` — CMake's `FetchContent` clones
  every third-party dependency (GLFW, GLAD, GLM, ImGui, GoogleTest, stb) via
  `git` on first configure, so `git` has to be on `PATH` too, not just a
  compiler/CMake.
- `pkg-config`.
- A Python 3 interpreter on `PATH` (`python3 --version`) **with the `jinja2`
  package installed** (`python3 -c "import jinja2"` to check). CMake configure
  runs GLAD's code generator, which is a Python script templated with Jinja2.

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

**FOR WINDOWS USERS**: Untested since the refactor. GLFW itself supports Windows
fine; you're on your own for the exact dependency setup. Feel free to find out
and open a PR making changes to these instructions.

**FOR MACOS USERS**: The renderer targets OpenGL 4.1 core instead of a newer
version because Apple's OpenGL implementation has been frozen at 4.1. That said,
it hasn't been built/tested on macOS yet since the rewrite. If you're on macOS,
feel free to find out and open a PR making changes to these instructions.

## Testing

Unit tests live in `tests/` and cover everything outside `src/render/` (math,
shapes/collision, broadphase, narrowphase/GJK/EPA, solver/dynamics, fixed
timestep) with GoogleTest.

Configure and build the test binary:

```
cmake -B build -S .
cmake --build build --target rp_tests -j
```

Then run the tests, either directly:

```
./build/bin/rp_tests
```

or through ctest, for per-case pass/fail output:

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

Other knobs set in code today (none exposed in ImGui panels yet, look at
[TODO](#todo)):

- `World::setGravity(Vec3f)`: per-axis gravity vector.
- `World::addWorldBoundaries(worldMin, worldMax, thickness, friction,
  restitution, is2D)`: builds a box of static `BoxShape` walls.
- `World::setBroadphase(std::unique_ptr<IBroadphase>)`: swap
  `NaiveBroadphase`/`GridBroadphase`/`DynamicBVHBroadphase` (default) at
  runtime.
- `World::config()` returns a mutable `SolverConfig`: velocity/position solver
  iteration counts, allowed penetration slop, max per-iteration position
  correction, and the Baumgarte stabilization factor.
- `Simulator`'s `SimConfig`: `fixedDt` (physics step size) and
  `maxStepsPerFrame` (runaway-accumulator clamp).

## Controls

The sidebar panel (`ImguiController`) is organized by section, and this is
everything it currently exposes:

**Camera** — switch between:
- **Orthographic (2D)**: Left-drag the world view to pan. No zoom control
  yet.
- **Perspective (3D)**: Click the world view to capture the mouse, then look
  around with the mouse and move with W/A/S/D (Space/Ctrl for up/down), all
  usable together. Esc releases the cursor.

**Spawn** — "Spawn Sphere"/"Spawn Box"/"Spawn Capsule" each drop one body of
that shape at a randomized position above the scene. "Reset Scene" removes every
dynamic body, leaving statics in place.

**Screenshot** — "Take Screenshot" opens a path prompt and writes the current
framebuffer to a PNG (this is how the screenshot above was captured).

There's currently no force-application tool and no runtime
gravity/restitution/broadphase/ narrowphase controls, but will be added in the
future.

## State of Simulation Performance

Haven't yet re-benchmarked. The old numbers are irrelevant now that there's a
GPU-instanced OpenGL pipeline. A proper Debug/Release benchmark against the
100k-body target still needs to be done.

## TODO

- [ ] add multithreading (broadphase makes a flat pair/manifold list per fixed
  step, can parallelize here)
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
- [ ] re-benchmark simulation performance
- [ ] refactor
  - [ ] turn physics code into a separate library
  - [ ] turn rendering code into a separate library
  - [ ] rebuild RPEngine using this library approach
