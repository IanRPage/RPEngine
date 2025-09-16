# RPEngine2D

## TODO

- [x] implement working `QuadTree`
- [x] make custom `AABB` struct independent of SFML
- [x] change implementation `QuadTree` to use `AABB` instead of SFML `FloatRect`
- [x] optimize number capacity of `QuadTree` partition
- [x] move UI stuff into its own rendering engine class
- [x] decouple simulation code from UI code
- [x] implement proper resizing
- [ ] add a Verlet integration based resolver to `Simulator`
- [ ] add a "switch" for switching between Euler-Impulse and Verlet-Position based collisions
- [ ] implement spatial hashing
- [ ] change all graphics code to use OpenGL instead of SFML
