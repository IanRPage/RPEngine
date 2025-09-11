#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include <Simulator.hpp>
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
  sf::Clock frameClock_;
  sf::Vector2u lastSize_;

  // assets
  sf::Font font_;
  sf::Texture particleTexture_;

  // UI components
  HorizSlider gSlider_;
  HorizSlider eSlider_;
  sf::Text particleCountText_;
  sf::Text fpsText_;

  // other
  const float particleSize_ = 10.0f;
  bool draggingAny_ = false;

  void layoutUI();
  void handleMousePressed(const sf::Event::MouseButtonPressed &e);
  void handleMouseReleased();
  void handleMouseMoved(const sf::Event::MouseMoved &e);

  void drawParticles();
  void drawComponents();
  void updateText();
};

#endif
