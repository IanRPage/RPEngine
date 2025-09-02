#include <SFML/Graphics.hpp>
#include <Simulator.h>
#include <ui/Slider.h>

const float SCALE = 0.86f;

int main() {
  // make window SCALE% of screen resolution
  sf::Vector2f windowDims =
      static_cast<sf::Vector2f>(sf::VideoMode::getDesktopMode().size);
  windowDims *= SCALE;
  sf::Vector2u dim = static_cast<sf::Vector2u>(windowDims); // final dimensions
  sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(dim), "RPEngine");
  window.setFramerateLimit(60);

  const float dt = 1.0f / 60.0f; // time delta

  Simulator sim(dim, 0.0f, 1.0f, dt);
  HorizSlider gSlider({40.0f, windowDims.y - 100.0f}, {200.0f, 10.0f},
                  {-100.0f, 100.0f}, sim.gravity);
  HorizSlider c_rSlider({40.0f, windowDims.y - 50.0f}, {200.0f, 10.0f},
                    {0.0f, 1.0f}, sim.restitution, sf::Color::White,
                    sf::Color::Yellow);

  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
      }
      if (const auto *mousePressed =
              event->getIf<sf::Event::MouseButtonPressed>()) {
        if (mousePressed->button == sf::Mouse::Button::Left) {
          if (gSlider.contains(mousePressed->position)) {
            gSlider.isDragging = true;
          } else if (c_rSlider.contains(mousePressed->position)) {
            c_rSlider.isDragging = true;
          } else {
            sim.spawnParticle(mousePressed->position);
          }
        }
      }

      if (event->getIf<sf::Event::MouseButtonReleased>()) {
        if (gSlider.isDragging == true) {
          gSlider.isDragging = false;
        } else if (c_rSlider.isDragging == true) {
          c_rSlider.isDragging = false;
        }
      }

      if (gSlider.isDragging) {
        if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
          gSlider.move(mouseMoved->position);
        }
      }
      if (c_rSlider.isDragging) {
        if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
          c_rSlider.move(mouseMoved->position);
        }
      }
    }
    window.clear();
    sim.update();
    for (auto &par : sim.getParticles()) {
      window.draw(par.shape);
    }
    gSlider.draw(window);
    c_rSlider.draw(window);
    window.display();
  }
}
