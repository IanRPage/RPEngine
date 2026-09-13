# Resources

This is a brief list of resources I based my physics, collision, and rendering
code on. These are solid references for understanding the algorithms and
concepts behind code. There is WAY more than I could possible list here, but
these are great starting points.

## Narrowphase collision (GJK + EPA)

- [Erin Catto, "Computing Distance using GJK" (GDC 2010)](https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf)
- [Casey Muratori, "Implementing GJK - 2006"](https://www.youtube.com/watch?v=Qupqu1xe7Io)
- [Dirk Gregorius, "The Separating Axis Test Between Convex Polyhedra" (GDC 2013 slides)](https://media.gdcvault.com/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf)
  - SAT as a companion/alternative to GJK for polytope overlap
- [Dirk Gregorius, "Implementing QuickHull" (GDC 2014 slides)](http://media.steampowered.com/apps/valve/2014/DirkGregorius_ImplementingQuickHull.pdf)
  - convex hull construction, was useful background for EPA's expanding polytope

## Solver (sequential impulse / PGS)

- [Erin Catto, "Iterative Dynamics with Temporal Coherence" (GDC 2005)](https://box2d.org/files/ErinCatto_IterativeDynamicsSlides_GDC2005.pdf)
  - covers warm-starting and bias-velocity
- [Erin Catto, "Soft Constraints" (GDC 2011)](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf)
- [Erin Catto, "Modeling and Solving Constraints" (GDC 2009)](https://box2d.org/files/ErinCatto_ModelingAndSolvingConstraints_GDC2009.pdf)
  - Baumgarte stabilization
- [Box2D source](https://github.com/erincatto/box2d)
- [Bullet Physics source](https://github.com/bulletphysics/bullet3)
  - `btSequentialImpulseConstraintSolver.cpp` ideas applied in `Solver.cpp`

## Broadphase (Dynamic BVH)

- [Eric Catto, "Dynamic Bounding Volume Hierarchies" (GDC 2019)](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf)

## Integration & rigid body / mass properties

- [Baraff & Witkin, "Physically Based Modeling: Rigid Body Simulation" (Pixar course notes, 2001)](https://web.mat.upc.edu/toni.susin/files/BaraffWitkinKass2001.pdf)
  - this one was absolute gold. everyone should read this
  - a bit more organized version (with less content): [Baraff, CMU SIGGRAPH course notes ("An Introduction to Physically Based Modeling")](https://www.cs.cmu.edu/~baraff/sigcourse/)
- [GameDev.net, "Capsule Inertia Tensor"](https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/)
  - capsule inertia math

## Math (glm)

- [glm documentation](https://glm.g-truc.net/0.9.9/index.html) / [glm GitHub](https://github.com/g-truc/glm)

## Rendering

`src/render/*`

- [learnopengl.com](https://learnopengl.com/)
- [Anton Gerdelan's free OpenGL tutorials](https://antongerdelan.net/opengl/)
- [Song Ho Ahn, "OpenGL Projection Matrix"](https://www.songho.ca/opengl/gl_projectionmatrix.html)
- [Song Ho Ahn, "OpenGL Transformation"](https://www.songho.ca/opengl/gl_transform.html)
- [Khronos Wiki, "Vertex Rendering" (Instancing section)](https://www.khronos.org/opengl/wiki/Vertex_Rendering#Instancing)
- [GLFW documentation](https://www.glfw.org/documentation.html)
- [Khronos OpenGL Wiki](https://www.khronos.org/opengl/wiki/)

---

NOTE: *Real-Time Collision Detection* (Ericson) and *Game Physics Engine
Development* (Millington) are super solid books that helped me out, but both are
paid. The above resources are great alternatives for free.
