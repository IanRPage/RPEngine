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

  // platform 1
  world.createStaticBody(
      BoxShape{Vec3f(13.5f, 0.5f, 13.5f)},
      Transform{Vec3f(0.0f, -0.5f, 0.0f), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 0.5f,
      0.3f);

  // platform 2
  BodyHandle platform2 = world.createStaticBody(
      BoxShape{Vec3f(27.0f, 0.5f, 27.0f)},
      Transform{Vec3f(0.0f, -5.0f, 0.0f), Quatf(1.0f, 0.0f, 0.0f, 0.0f)}, 0.5f,
      0.3f);
  renderables.setColor(platform2, Vec4f(0.35f, 0.35f, 0.35f, 1.0f));

  for (int i = 0; i < 300; ++i) {
    float x = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float z = static_cast<float>(std::rand() % 600 - 300) / 100.0f;
    float y = 7.0f + static_cast<float>(i) * 0.5f;
    Transform t{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)};

    BodyHandle handle;
    switch (i % 3) {
      case 0:
        handle =
            world.createDynamicBody(SphereShape{0.5f}, t, 1.0f, 0.5f, 0.5f);
        break;
      case 1:
        handle = world.createDynamicBody(BoxShape{Vec3f(0.5f, 0.5f, 0.5f)}, t,
                                         1.0f, 0.5f, 0.3f);
        break;
      default:
        handle = world.createDynamicBody(CapsuleShape{0.35f, 0.45f}, t, 1.0f,
                                         0.5f, 0.4f);
        break;
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
