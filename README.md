# RPEngine2D

## TODO

- [x] implement working `QuadTree`
- [x] make custom `AABB` struct independent of SFML
- [x] change implementation `QuadTree` to use `AABB` instead of SFML `FloatRect`
- [x] optimize number capacity of `QuadTree` partition
- [x] move UI stuff into its own rendering engine class
- [x] decouple simulation code from UI code
- [x] implement proper resizing
- [x] add a Verlet integration based resolver to `Simulator`
- [x] implement spatial grid broad-phase
- [x] add debug and release builds
- [x] implement SpatialGrid class and move some stuff out of
  `Simulator::spatialGridBroadphase()`
- [ ] add multithreading
- [ ] add cool things that can be done with clicks
- [ ] add rigidbody mechanics
- [ ] optimize spatial grid broad-phase
- [ ] MAYBE improve wall collision code by only checking particles along the
  walls or something
- [ ] add a UI option for toggling between Euler-Impulse and Verlet-Position
  based collisions
- [ ] add a UI option for toggling between broad phase methods for collision
  detection
- [ ] add some kind of profiler that runs a simulation without UI
