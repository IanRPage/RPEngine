#include <Simulator.hpp>
#include <imgui-SFML.h>
#include <ui/Renderer.hpp>

int main() {
  Simulator sim({0.0f, 0.0f}, 2.0f, 0.0f, 0.65f, 0.0, IntegrationType::Verlet,
                BroadphaseType::SpatialGrid, 55000);
  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  renderer.mainLoop();

  return 0;
}
