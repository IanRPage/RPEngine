#include <SFML/Graphics.hpp>
#include <Simulator.h>

const float SCALE = 0.86f;

int main() {
  // make window SCALE% of screen resolution
  sf::Vector2f windowDims =
      static_cast<sf::Vector2f>(sf::VideoMode::getDesktopMode().size);
  windowDims *= SCALE;
  sf::Vector2u dim = static_cast<sf::Vector2u>(windowDims); // final dimensions
  sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(dim), "RPEngine");
  window.setFramerateLimit(60);

  float g = 9.8f;                // gravity
  float C_r = 0.95f;             // coefficient of restitution
  const float dt = 1.0f / 60.0f; // time delta

  Simulator sim(dim, g, C_r, dt);

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
    sim.update();
    for (auto &par : sim.getParticles()) {
      window.draw(par.shape);
    }
    window.display();
  }
}
