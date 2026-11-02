#include <collision/Shapes.hpp>
#include <core/World.hpp>
#include <gui/RenderableStore.hpp>
#include <gui/Renderer.hpp>
#include <math/Transform.hpp>
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

void buildDemoScene(Simulator& sim, gui::RenderableStore& renderables) {
  World& world = sim.world();

  CompoundShape openBox({
      CompoundChild{BoxShape{Vec3f(8.5f, 0.5f, 8.5f)},
                    Transform{Vec3f(0.0f, -0.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(0.5f, 3.5f, 8.5f)},
                    Transform{Vec3f(-8.0f, 3.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(0.5f, 3.5f, 8.5f)},
                    Transform{Vec3f(8.0f, 3.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(7.5f, 3.5f, 0.5f)},
                    Transform{Vec3f(0.0f, 3.5f, -8.0f)}},
      CompoundChild{BoxShape{Vec3f(7.5f, 3.5f, 0.5f)},
                    Transform{Vec3f(0.0f, 3.5f, 8.0f)}},
  });
  world.createStaticBody(std::move(openBox), Transform{}, 0.5f, 0.3f);

  CompoundShape hammer =
      CompoundShape({
          CompoundChild{BoxShape{Vec3f(0.15f, 0.6f, 0.15f)}, Transform{}, 1.0f},
          CompoundChild{BoxShape{Vec3f(0.5f, 0.2f, 0.25f)},
                        Transform{Vec3f(0.0f, 0.8f, 0.0f)}, 3.0f},
          CompoundChild{SphereShape{0.25f},
                        Transform{Vec3f(0.0f, -0.75f, 0.0f)}, 1.0f},
      })
          .recentered();

  for (int i = 0; i < 1000; ++i) {
    float x = static_cast<float>(std::rand() % 800 - 400) / 100.0f;
    float z = static_cast<float>(std::rand() % 800 - 400) / 100.0f;
    float y = 7.0f + static_cast<float>(i) * 0.5f;
    Transform t{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)};

    BodyHandle handle;
    switch (i % 4) {
      case 0:
        handle =
            world.createDynamicBody(SphereShape{0.5f}, t, 1.0f, 0.5f, 0.5f);
        break;
      case 1:
        handle = world.createDynamicBody(BoxShape{Vec3f(0.5f, 0.5f, 0.5f)}, t,
                                         1.0f, 0.5f, 0.3f);
        break;
      case 2:
        handle = world.createDynamicBody(CapsuleShape{0.35f, 0.45f}, t, 1.0f,
                                         0.5f, 0.4f);
        break;
      default:
        handle = world.createDynamicBody(hammer, t, 2.0f, 0.5f, 0.3f);
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
    gui::RenderableStore renderables;
    buildDemoScene(sim, renderables);

    gui::Renderer renderer(sim, renderables);
    renderer.mainLoop();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
