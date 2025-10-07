# RPEngine

This is my attempt at building a particle simulator. I made this to practice
applying physics to code and to practice algorithms. But probably most of all
this just felt like a fun project to make. My goal is to be able to run the 2D
simulation with 100k particles at 60 fps.

## Building

The UI uses SFML 3.0.1, so make sure you have all the dependencies installed:

### Debian/Ubuntu:

```
sudo apt update
sudo apt install \
    libxrandr-dev \
    libxcursor-dev \
    libxi-dev \
    libudev-dev \
    libfreetype-dev \
    libflac-dev \
    libvorbis-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libfreetype-dev
```

### Fedora:

```
sudo dnf update
sudo dnf install \
    libXrandr-devel \
    libXcursor-devel \
    libXi-devel \
    systemd-devel \
    freetype-devel \
    flac-devel \
    libvorbis-devel \
    mesa-libGL-devel \
    mesa-libEGL-devel
```

### Arch:

```
sudo pacman -Sy
sudo pacman -S \
    libxrandr \
    libxcursor \
    libxi \
    systemd \
    freetype2 \
    flac \
    libvorbis \
    mesa
```

After all dependencies are installed, just configure the project as normal:

```
cmake -B build .  # from project root
cmake ..          # from build/
```

From here you can either build the debug (`RPEngineDebug`) or release
(`RPEngine`) executables:

```
# from project root
make -C build release  (configure for release and build)
make -C build debug    (configure for debug and build)

# from build/
make release
make debug
```

## Controls

There's only some pretty rudimentary controls at the moment:

- _Spawn a single particle_ -> **right click** anywhere on the screen.
- _Begin randomly spawning particles_ -> **press R**.
- _Randomly spawn particles 5x faster_ -> **press F**.
- _Spawn particle in an oscillating stream_ -> **press Space**.
- _Spawn `capacity` number of particle_ -> **press M**.
- _Radially push particles from mouse_ -> **hold fown left click**. you can move
  your mouse around and it will continue applying.

Apart from these controls there is also a gravity and coefficient of restitution
slider (describes how much kinetic energy is lost on collision) to manipulate
the simulation some more.

## Tuning the Simulation

In `main.cpp`, you'll find the initialization of the simulator. Here is an explanation of each parameter:

```cpp
Simulator sim(
  {0.0f, 0.0f},                     // window dimensions
  2.0f,                             // maximum particle radius
  0.0f,                             // magnitude of gravity
  0.0f,                             // magnitude of coefficient of restitution
  0.0,                              // delta time step for simulation
  IntegrationType::Verlet,          // integration type
  BroadphaseType::UniformGrid,      // broadphase type
  14000                             // max amount of particles allowed in the
                                    // simulator
);
```

The reason that a bunch of these parameters are set to 0 in `main` is because
the `Renderer` ends up manipulating them through its window size, and the
parameters get tuned during the simulation. The simulation can run independent
of the GUI.

## State of Simulation Performance

My laptop is an Asus VivoBook with AMD Ryzen 5800HS processor (integrated
graphics), 12 GB RAM. Currently

- w/ Verlet Integration:

  - Debug: 14k particles at 60 fps.
  - Release: 55k particles at 60 fps. 100k at ~31 fps.

- w/ Euler Integration:

  - Debug: 14k particles at 60 fps.
  - Release: 49k particle at 60 fps. 100k at ~28 fps

## TODO

- [ ] use ImGui to add controls for toggling between simulating different ways
- [ ] explain all controls in GUI once ImGui controls implemented
- [ ] add support for Winblows
- [ ] implement hot-reloading for quicker debugging
- [ ] add 3D particle simulation
- [ ] MAYBE add orbiting
- [ ] add multithreading
- [ ] add rigidbody mechanics
- [ ] optimize spatial grid broad-phase
- [ ] MAYBE improve wall collision code by only checking particles along the
      walls or something
- [ ] add a UI option for toggling between Euler-Impulse and Verlet-Position
      based collisions
- [ ] add a UI option for toggling between broad phase methods for collision
      detection
- [ ] add some kind of profiler that runs a simulation without UI
- [x] fix particles exploding when compacted w/ Verlet integration
- [x] add instructions for controls!!!
- [x] add radial push
- [x] change particle drawing to vertex-based
- [x] implement SpatialGrid class and move some stuff out of
      `Simulator::spatialGridBroadphase()`
- [x] add debug and release builds
- [x] implement spatial grid broad-phase
- [x] add a Verlet integration based resolver to `Simulator`
- [x] implement proper resizing
- [x] move UI stuff into its own rendering engine class
- [x] decouple simulation code from UI code
- [x] optimize number capacity of `QuadTree` partition
- [x] change implementation `QuadTree` to use `AABB` instead of SFML `FloatRect`
- [x] make custom `AABB` struct independent of SFML
- [x] implement working `QuadTree`
