#ifndef RENDERER_H
#define RENDERER_H

#define PI 3.14159265

#include <ui/ImguiController.hpp>
#include <SFML/Graphics.hpp>
#include <Simulator.hpp>
#include <array>

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
  void mainLoop();

  sf::Vector2u windowSize() const noexcept { return window_.getSize(); };

 private:
  Simulator& sim_;
  sf::RenderWindow window_;
  ImguiController imguiCtrl_;
  sf::Vector2u lastSize_;
  sf::Vector2f pushOrigin_;  // tracks mouse position when held down

  // timers
  sf::Clock frameClock_;
  sf::Clock spawnClock_;
  sf::Clock runtimeClock_;

  std::vector<std::optional<sf::Color>> colorLUT_;  // color lookup table

  // vertex based circle drawing
  sf::VertexArray particleVertices_;
  static constexpr size_t MIN_CIRCLE_SEGMENTS = 6;
  static constexpr size_t MAX_CIRCLE_SEGMENTS = 24;
  std::array<std::vector<sf::Vector2f>, MAX_CIRCLE_SEGMENTS + 1> unitCircle_;

  // --- other variables ---
  bool draggingAny_ = false;
  bool radialPushing_ = false;

  const float spawnInterval_ = 0.001f;
  bool streamSpawn_ = false;
  bool randomSpawn_ = false;
  bool randomSpawnSUPERFAST_ = false;
  bool spawnMax_ = false;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> distX;
  std::uniform_real_distribution<float> distY;

  // --- helpers ---
  void computeUnitCircle();
  size_t getCircleSegments(float radius);

  const sf::Color getRainbow(float t) noexcept;
  const sf::Color& colorFor(const Particle& p) noexcept;
  void layoutUI() noexcept;

  void handleMousePressed(const sf::Event::MouseButtonPressed& e) noexcept;
  void handleMouseReleased() noexcept;
  void handleMouseMoved(const sf::Event::MouseMoved& e) noexcept;
  void handleKeyPressed(const sf::Event::KeyPressed& e) noexcept;

  void drawParticles();

  void randomSpawn() noexcept;
  void randomSpawnSUPERFAST() noexcept;
  void streamSpawn() noexcept;
  void spawnMax() noexcept;
  void radialPush();
};

#endif
