#include <Simulator.h>

void Simulator::spawnParticle(sf::Vector2i pos) {
  Particle circle(static_cast<sf::Vector2f>(pos), {0.0f, 0.0f});
  particles.push_back(circle);
}

void Simulator::update() {
  for (Particle &par : particles) {
    par.accelerate({0.0f, gravity}); // apply gravity
    par.update(dt);
  }
}
