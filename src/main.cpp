#include <Simulator.hpp>
#include <ui/Renderer.hpp>

int main() {
  // NOTE: values set here in sim constructor will be changed by Renderer
  // Simulator sim({0.0f, 0.0f}, 5.0f, 0.0f, 0.0f, 0.0f, IntegrationType::Euler, 100000);
  Simulator sim({0.0f, 0.0f}, 2.0f, 0.0f, 0.0f, 0.0, IntegrationType::Verlet, 100000);

  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  while (renderer.isOpen()) {
    renderer.pollAndHandleEvents();
    sim.update();
    renderer.drawFrame();
  }
  return 0;
}
