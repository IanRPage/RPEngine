#include <collision/Shapes.hpp>
#include <core/World.hpp>
#include <math/Transform.hpp>
#include <render/RenderableStore.hpp>
#include <render/Renderer.hpp>
#include <sim/Simulator.hpp>

#include <cstdlib>
#include <ctime>
#include <iostream>

namespace {

Vec4f randomColor() noexcept {
  auto channel = []() {
    return static_cast<float>(std::rand() % 156 + 100) / 255.0f;
  };
  return Vec4f(channel(), channel(), channel(), 1.0f);
}

void buildDemoScene(Simulator& sim, render::RenderableStore& renderables) {
  World& world = sim.world();

  // 2D-constrained box arena, visible from the default orthographic view.
  world.addWorldBoundaries(Vec3f(-8.0f, -6.0f, -1.0f), Vec3f(8.0f, 6.0f, 1.0f),
                           0.5f, 0.5f, 0.3f, /*is2D=*/true);

  for (int i = 0; i < 12; ++i) {
    float x = static_cast<float>(i % 6) * 2.0f - 5.0f;
    float y = 2.0f + static_cast<float>(i / 6) * 2.0f;
    BodyHandle handle;
    if (i % 2 == 0) {
      handle = world.createDynamicBody(
          SphereShape{0.5f}, Transform{Vec3f(x, y, 0.0f), Quatf(1, 0, 0, 0)},
          1.0f, 0.5f, 0.6f, /*constrainTo2D=*/true);
    } else {
      handle = world.createDynamicBody(
          BoxShape{Vec3f(0.5f, 0.5f, 0.5f)},
          Transform{Vec3f(x, y, 0.0f), Quatf(1, 0, 0, 0)}, 1.0f, 0.5f, 0.4f,
          /*constrainTo2D=*/true);
    }
    renderables.setColor(handle, randomColor());
  }
}

}  // namespace

int main() {
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  try {
    Simulator sim;
    render::RenderableStore renderables;
    buildDemoScene(sim, renderables);

    render::Renderer renderer(sim, renderables);
    renderer.mainLoop();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
