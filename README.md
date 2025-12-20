# RPEngine

This is my attempt at building a particle simulator. I made this to practice
applying physics to code and to practice algorithms. But probably most of all
this just felt like a fun project to make. My goal is to be able to run the 2D
simulation with 100k particles at 60 fps.

I will eventually add a 3D mode to the simulation and GUI portion for cooler
stuff.

![demo1](images/demo1.gif?raw=true)

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

**FOR WINDOWS USERS**: I got it running on my windows system, but I had already
gone through getting SFML to work prior. Tbh, I don't remember what I did. That
said, I got the simulation running on windows. You're on your own here. Feel
free to find out and open a PR making changes to these instructions.

**FOR MACOS USERS**: I don't have access to a mac, so idk what is needed to get
it running. That said, macOS is close to unix so I'd imagine the same commands
for getting it running on Ubuntu/Debian would be nearly the same. Feel free to
find out and open a PR making changes to these instructions.

## Tuning the Simulation

In `main.cpp`, you'll find the initialization of the simulator. Here is an
explanation of each parameter:

```cpp
Simulator sim(
  {0.0f, 0.0f},                     // container dimensions
  50.0f,                            // maximum particle radius
  0.0f,                             // magnitude of gravity
  0.0f,                             // magnitude of coefficient of restitution
  0.0f,                             // delta time step for simulation
  14000,                            // max amount of particles allowed in the
                                    // simulator
  IntegrationType::Verlet,          // integration type
  BroadphaseType::SpatialGrid,      // broadphase type
);
```

Again, these are the initialization parameters for the `Simulator` class. The
reason I leave these parameters is because eventually I will enable running the
simulation without the GUI, it will take in particle data and it will stream
particle data out to different files. That will take some time but yeah.

The reason that a bunch of these parameters are set to garbage values in
`main.cpp` is because the `Renderer` ends up manipulating them upon
initialization.

- NOTE: only `maxParticleRadius` and `maxParticles` don't get changed by the
  `Renderer`. So you do need to think about these ones.

You can further tune the simulation at runtime by using the panel below:

![panel1](images/panel1.gif?raw=true)

## Controls

To apply a force:

1. Select the force you want.

2. Left-click to apply it; the force will be applied for as long as you hold
   down the left mouse.

To spawn particles:

1. Select the spawning method.

2. If you selected **Manual** spawn, you right-click to activate it. For all
   other spawning methods, press \<Space\> to toggle it on/off.

If you forget these instructions, the Help dropdown in the panel will tell you:

![panel2](images/panel2.gif?raw=true)

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

- [ ] look into forward declarations for certain classes that rely on others
- [ ] make a callback system for event handling. right now `Renderer::drawFrame`
      is responsible for executing events
- [ ] add `gtest` testing suite to ensure physical accuracy
- [ ] see about optimizing `Renderer::drawParticles`
- [ ] rearchitect the codebase (cuz why not)
- [ ] make transition between integration types clean. simulation crashes from
      Euler -> Verlet
- [ ] improve `Simulator::radialPush` to work with any broadphase
- [ ] implement hot-reloading for quicker debugging
- [ ] add 3D particle simulation
- [ ] MAYBE add orbiting
- [ ] add multithreading
- [ ] add rigidbody mechanics
- [ ] MAYBE improve wall collision code by only checking particles along the
      walls or something
- [ ] add some kind of profiler that runs a simulation without UI
- [ ] optimize spatial grid broadphase
  - [ ] do a different broadphase for differently sized particles
  - [x] fix particle collision instability; they violate particle bounds A LOT
  - [x] fix massive performance degradation in `SpatialGrid` when lots of small
        particles but large cell size
  - [x] fix particle rightward drift during `SpatialGrid` broadphase when
        tightly packed
- [x] use ImGui to enable simulation configuration
  - [x] make the panel cleaner looking
  - [x] explain all controls in GUI once ImGui controls implemented
  - [x] add spawning methods to ImGui
  - [x] add forces options
  - [x] add magnitude param for `radialPush`
  - [x] make it so that forces activated on mouse click don't get applied when
        hovering over the ImGui menu
  - [x] add radius param for `radialPush`
  - [x] add particle radius field
  - [x] add a UI option for toggling between Euler-Impulse and Verlet-Position
        based collisions
  - [x] add a UI option for toggling between broad phase methods for collision
        detection
- [x] fix the downsizing radius issue
- [x] fix particles exploding when compacted w/ Verlet integration
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
