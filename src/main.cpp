#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <Simulator.h>

const float G = 980.0f;
const float RESTITUTION = 0.95f; // the dampening effect of bounce
const float SCALE = 0.86f;

int main() {
  // make window SCALE% of screen resolution
  sf::Vector2f windowDims =
      static_cast<sf::Vector2f>(sf::VideoMode::getDesktopMode().size);
  windowDims *= SCALE;
  sf::Vector2u dim = static_cast<sf::Vector2u>(windowDims); // final dimensions
  sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(dim), "RPEngine");
  window.setFramerateLimit(60);

  Simulator sim(dim);
  const float dt = 1.0f / 60.0f;

  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }
      if (const auto *mousePressed =
              event->getIf<sf::Event::MouseButtonPressed>()) {
        if (mousePressed->button == sf::Mouse::Button::Left) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          sim.spawnParticle(mousePos);
        }
      }
    }
    window.clear();

    for (auto &par : sim.particles) {
      par.setPosition(par.position);
      par.velocity.y += G * dt;
      par.position += par.velocity * dt;

      auto [maxX, maxY] = sim.windowDims;
      float radius = par.getRadius();

      // top edge
      if (par.position.y < 0) {
        par.position.y = 0;
        par.velocity.y = -par.velocity.y * RESTITUTION;
      }

      // bottom edge
      if (par.position.y + 2 * radius > maxY) {
        par.position.y = maxY - 2 * radius;
        par.velocity.y = -par.velocity.y * RESTITUTION;
      }

      // left edge
      if (par.position.x < 0) {
        par.position.x = 0;
        par.velocity.x = -par.velocity.x * RESTITUTION;
      }

      // right edge
      if (par.position.x + 2 * radius > maxX) {
        par.position.x = maxX - 2 * radius;
        par.velocity.x = -par.velocity.x * RESTITUTION;
      }

      window.draw(par);
    }

    window.display();
  }
}
