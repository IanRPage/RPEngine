#include <Simulator.hpp>
#include <ui/Renderer.hpp>

int main() {
  Simulator sim;

  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  while (renderer.isOpen()) {
    renderer.pollAndHandleEvents();
    sim.update();
    renderer.drawFrame();
  }
  return 0;
}
