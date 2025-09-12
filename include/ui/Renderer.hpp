#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include <Simulator.hpp>
#include <random>
#include <string>
#include <ui/Slider.hpp>

class Renderer {
public:
  struct Options {
    unsigned fps_limit;
    std::string window_title;
  };

  Renderer(Simulator &sim, const Options &opts = {60, "RPEngine"});
  ~Renderer() = default;

  bool isOpen() const { return window_.isOpen(); };
  void pollAndHandleEvents();
  void drawFrame();

  sf::Vector2u windowSize() const { return window_.getSize(); };

private:
  Simulator &sim_;
  sf::RenderWindow window_;
  sf::Vector2u lastSize_;
  sf::Clock frameClock_;
  sf::Clock spawnClock_;
  sf::Clock runtimeClock_;

  // assets
  sf::Font font_;
  sf::Texture particleTexture_;

  // UI components
  HorizSlider gSlider_;
  HorizSlider eSlider_;
  sf::Text particleCountText_;
  sf::Text fpsText_;
  sf::Sprite particleSprite_;

  // other variables
  const float particleSize_ = 5.0f;
  bool draggingAny_ = false;

  const float spawnInterval_ = 0.001f;
  bool streamSpawn_ = false;
  bool randomSpawn_ = false;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> distX;
  std::uniform_real_distribution<float> distY;

  void layoutUI();
  void handleMousePressed(const sf::Event::MouseButtonPressed &e);
  void handleMouseReleased();
  void handleMouseMoved(const sf::Event::MouseMoved &e);
  void handleKeyPressed(const sf::Event::KeyPressed &e);

  void drawParticles();
  void drawComponents();
  void updateText();

  void randomSpawn();
  void streamSpawn();
};

#endif
