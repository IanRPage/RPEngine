#include <SFML/Graphics.hpp>
#include <Simulator.h>
#include <stdexcept>
#include <ui/Slider.h>

// TODO move UI code into a separate renderer
const float PARTICLESIZE = 5.0f;

int main() {
	sf::Vector2u dim = sf::VideoMode::getDesktopMode().size;
  sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(dim), "RPEngine");
  window.setFramerateLimit(60);
  sf::Vector2u actualWindowSize = window.getSize();

  sf::Clock clock;
  sf::Font font;
  if (!font.openFromFile("assets/roboto.ttf")) {
    return -1;
  }
  sf::Text particleCount(font, "Particles: 0", 30);
  sf::Text fpsText(font, "FPS: ", 30);
  particleCount.setPosition({10, 0});
  fpsText.setPosition({10, 30});

  sf::Texture particleTexture;
  if (!particleTexture.loadFromFile("assets/particle1.png")) {
    throw std::invalid_argument("Texture: invalid texture name");
  }

  const float dt = 1.0f / 60.0f; // time delta

  bool autoSpawn = false;
  const int maxParticles = 100000;
  sf::Clock spawnTimer;
  const float spawnInterval = 0.01f;
  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<float> distX(0.0f, actualWindowSize.x - 20.0f);
  std::uniform_real_distribution<float> distY(0.0f, actualWindowSize.y - 20.0f);

  Simulator sim({actualWindowSize.x, actualWindowSize.y}, 0.0f, 1.0f, dt,
                maxParticles);

  HorizSlider gSlider({40.0f, actualWindowSize.y - 100.0f}, {200.0f, 10.0f},
                      {-100.0f, 100.0f}, sim.gravity);
  HorizSlider eSlider({40.0f, actualWindowSize.y - 50.0f}, {200.0f, 10.0f},
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
          } else if (eSlider.contains(mousePressed->position)) {
            eSlider.isDragging = true;
          } else {
            sf::Vector2i pos = mousePressed->position;
            sim.spawnParticle(Vec2f(pos.x, pos.y), PARTICLESIZE, 1.0f,
                              &particleTexture);
          }
        }
      }

      if (event->is<sf::Event::MouseButtonReleased>()) {
        if (gSlider.isDragging == true) {
          gSlider.isDragging = false;
        } else if (eSlider.isDragging == true) {
          eSlider.isDragging = false;
        }
      }

      if (const auto *keyPressed = event->getIf<sf::Event::KeyPressed>()) {
        if (keyPressed->scancode == sf::Keyboard::Scan::Space) {
          autoSpawn = !autoSpawn;
        }
      }

      if (gSlider.isDragging) {
        if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
          gSlider.move(mouseMoved->position);
        }
      }
      if (eSlider.isDragging) {
        if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
          eSlider.move(mouseMoved->position);
        }
      }
    }

    if (autoSpawn &&
        static_cast<int>(sim.getParticles().size()) < maxParticles &&
        spawnTimer.getElapsedTime().asSeconds() >= spawnInterval) {
      sf::Vector2f randPos(distX(gen), distY(gen));
      // sim.spawnParticle(static_cast<sf::Vector2i>(randPos),
      // &particleTexture);
      sim.spawnParticle({randPos.x, randPos.y}, PARTICLESIZE, 1.0f,
                        &particleTexture);
      spawnTimer.restart();
    }
    sim.update();

    window.clear();
    for (auto &par : sim.getParticles()) {
      window.draw(par.shape);
    }
    gSlider.draw(window);
    eSlider.draw(window);

    sf::Time elapsed = clock.restart();
    float fps = 1.0f / elapsed.asSeconds();
    fpsText.setString("FPS: " + std::to_string(static_cast<int>(fps)));
    particleCount.setString("Particles: " +
                            std::to_string(sim.getParticles().size()));

    window.draw(particleCount);
    window.draw(fpsText);

    window.display();
  }
}
