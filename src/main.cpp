#include <Simulator.hpp>
#include <ui/Renderer.hpp>

int main() {
  // NOTE: values set here in sim constructor will be changed by Renderer
  // Simulator sim({0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, IntegrationType::Euler);
  Simulator sim({0.0f, 0.0f}, 0.0f, 0.0f, 0.0, IntegrationType::Verlet);

  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  while (renderer.isOpen()) {
    renderer.pollAndHandleEvents();
    sim.update();
    renderer.drawFrame();
  }
  return 0;
}
