#include <imgui-SFML.h>

#include <Simulator.hpp>
#include <ui/Renderer.hpp>

int main() {
  Simulator sim({0.0f, 0.0f}, 50.0f, 0.0f, 0.65f, 0.01f, 50000);
  Renderer renderer(sim, Renderer::Options{60, "RPEngine"});

  renderer.mainLoop();

  return 0;
}
