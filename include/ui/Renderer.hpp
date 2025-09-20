#ifndef RENDERER_H
#define RENDERER_H

#define PI 3.14159265

#include <SFML/Graphics.hpp>
#include <Simulator.hpp>
#include <array>
#include <ui/Slider.hpp>

class Renderer {
 public:
  struct Options {
    unsigned fps_limit;
    std::string window_title;
  };

  Renderer(Simulator& sim, const Options& opts = {60, "RPEngine"});
  ~Renderer() = default;

  bool isOpen() const noexcept { return window_.isOpen(); };
  void pollAndHandleEvents() noexcept;
  void drawFrame();

  sf::Vector2u windowSize() const noexcept { return window_.getSize(); };

 private:
  Simulator& sim_;
  sf::RenderWindow window_;
  sf::Vector2u lastSize_;
  sf::Clock frameClock_;
  sf::Clock spawnClock_;
  sf::Clock runtimeClock_;
  std::unordered_map<uint32_t, sf::Color> colorLUT_;

  // assets
  sf::Font font_;
  sf::Texture particleTexture_;

  // UI components
  HorizSlider gSlider_;
  HorizSlider eSlider_;
  sf::Text particleCountText_;
  sf::Text fpsText_;
  sf::CircleShape particleShape_;

  // other variables
  float particleSize_ = 5.0f;
  bool draggingAny_ = false;

  const float spawnInterval_ = 0.001f;
  bool streamSpawn_ = false;
  bool randomSpawn_ = false;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> distX;
  std::uniform_real_distribution<float> distY;

  // fps measurement
  static constexpr size_t FPS_SAMPLE_COUNT = 60;
  std::array<float, FPS_SAMPLE_COUNT> frameTimes_;
  size_t frameIndex_ = 0;
  bool samplesCollected_ = false;

  // helper functions
  const sf::Color getRainbow(float t) noexcept;
  const sf::Color& colorFor(const Particle& p) noexcept;
  void layoutUI() noexcept;

  void handleMousePressed(const sf::Event::MouseButtonPressed& e) noexcept;
  void handleMouseReleased() noexcept;
  void handleMouseMoved(const sf::Event::MouseMoved& e) noexcept;
  void handleKeyPressed(const sf::Event::KeyPressed& e) noexcept;

  void drawParticles();
  void drawComponents();
  void updateText() noexcept;

  void randomSpawn() noexcept;
  void streamSpawn() noexcept;
};

#endif
