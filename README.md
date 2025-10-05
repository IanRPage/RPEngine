# RPEngine

This is my attempt at building a particle simulator. I made this to practice
applying physics to code and to practice algorithms. But probably most of all
this just felt like a fun project to make. My goal is to be able to run the 2D
simulation with 100k particles at at least 60 fps.

## Building

The UI uses SFML 3.0.1, so make sure you have all the dependencies installed:

Debian/Ubuntu:
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

Fedora:
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

After all the dependencies are installed, run the following commands from the
root to configure and build the project in debug mode:
```
cmake -B build .
make -C build
```
The executable will be called `RPEngineDebug`.

If you want to build the release version, simply use the CMAKE_BUILD_TYPE flag
when configuring with cmake. You do the following to configure for release:
```
cmake -DCMAKE_BUILD_TYPE=release -B build .
```
And then run `make` as normal. The release executable is called `RPEngine`.

## State of Simulation Performance
My laptop is an Asus VivoBook, AMD Ryzen 5800HS, 12 GB RAM. Currently, the debug
build can handle about 13k particles at 60 fps on it, and the release build can
handle about 55k particles at 60 fps, 100k particles at about 31 fps on it.

## Controls
There's only some pretty rudimentary controls at the moment:

- _Spawn a single particle_ -> __right click__ anywhere on the screen.
- _Begin randomly spawning particles_ -> __press R__.
- _Randomly spawn particles 5x faster_ -> __press F__.
- _Spawn particle in an oscillating stream_ -> __press Space__.
- _Spawn `capacity` number of particle_ -> __press M__.
- _Radially push particles from mouse_ -> __hold fown left click__. you can move
  your mouse around and it will continue applying.

Apart from these controls there is also a gravity and coefficient of restitution
slider (describes how much kinetic energy is lost on collision) to manipulate
the simulation some more.

## TODO
- [ ] __NEXT__ fix particles exploding when compacted w/ Verlet integration
- [ ] use ImGui to add controls for toggling between simulating different ways
- [ ] add support for Winblows
- [ ] implement hot-reloading for quicker debugging
- [ ] add 3D particle simulation
- [ ] BIG add instructions for controls!!!
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
