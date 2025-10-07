#include <Simulator.hpp>
#include <ui/Renderer.hpp>

int main() {
  // NOTE: running sim w/ Euler integration makes some really cool fx
  // Simulator sim({0.0f, 0.0f}, 2.0f, 0.0f, 0.0f, 0.0f, IntegrationType::Euler,
  //               BroadphaseType::UniformGrid, 12000);
  Simulator sim({0.0f, 0.0f}, 2.0f, 0.0f, 0.0f, 0.0, IntegrationType::Verlet,
                BroadphaseType::UniformGrid, 12000);

  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  while (renderer.isOpen()) {
    renderer.pollAndHandleEvents();
    sim.update();
    renderer.drawFrame();
  }
  return 0;
}
