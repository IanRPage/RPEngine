#include <stdexcept>
#include <ui/Renderer.hpp>

Renderer::Renderer(Simulator &sim, const Options &opts)
    : sim_(sim), window_(sf::RenderWindow(
                     sf::VideoMode(sf::VideoMode::getDesktopMode().size),
                     opts.window_title)),
      gSlider_({40.0f, 0.0f}, {200.0f, 10.0f}, {-100.0f, 100.0f}, sim_.gravity),
      eSlider_({40.0f, 0.0f}, {200.0f, 10.0f}, {0.0f, 1.0f}, sim_.restitution,
               sf::Color::White, sf::Color::Yellow),
      particleCountText_(font_, "Particles: ", 30),
      fpsText_(font_, "FPS: 60", 30) {
  window_.setFramerateLimit(opts.fps_limit);
  lastSize_ = window_.getSize();
  sim_.setWorldSize(
      Vec2f(static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)));

  // load font
  if (!font_.openFromFile("assets/pixelated.ttf")) {
    throw std::runtime_error("Failed to load font: assets/pixelated.ttf");
  };

  // load texture
  if (!particleTexture_.loadFromFile("assets/particle1.png")) {
    throw std::runtime_error("Failed to load texture: assets/particle1.png");
  };

  layoutUI();
}

void Renderer::layoutUI() {
  const auto size = window_.getSize();
  lastSize_ = size;
  sim_.setWorldSize(
      Vec2f(static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)));

  const float btnMargin = 40.0f;
  const float sliderWidth = 200.0f;
  const float sliderHeight = 10.0f;

  // button heights
  const float y1 = static_cast<float>(size.y) - 100.0f;
  const float y2 = static_cast<float>(size.y) - 50.0f;

  gSlider_.setPosition({btnMargin, y1});
  gSlider_.setSize({sliderWidth, sliderHeight});

  eSlider_.setPosition({btnMargin, y2});
  eSlider_.setSize({sliderWidth, sliderHeight});

  // top corner text
  particleCountText_.setPosition({5.0f, 0.0f});
  float fpsTextWidth = fpsText_.getLocalBounds().size.x;
  fpsText_.setPosition(
      {static_cast<float>(size.x) - fpsTextWidth - 5.0f, 0.0f});
}

void Renderer::pollAndHandleEvents() {
  while (const std::optional event = window_.pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
      window_.close();
    } else if (const auto *mousePressed =
                   event->getIf<sf::Event::MouseButtonPressed>()) {
      handleMousePressed(*mousePressed);
    } else if (event->is<sf::Event::MouseButtonReleased>()) {
      handleMouseReleased();
    } else if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
      handleMouseMoved(*mouseMoved);
    }
  }
}

void Renderer::handleMousePressed(const sf::Event::MouseButtonPressed &e) {
  if (e.button != sf::Mouse::Button::Left)
    return;
  if (gSlider_.contains(e.position)) {
    gSlider_.isDragging = true;
    draggingAny_ = true;
  } else if (eSlider_.contains(e.position)) {
    eSlider_.isDragging = true;
    draggingAny_ = true;
  } else {
    sf::Vector2i pos = e.position;
    sim_.spawnParticle(Vec2f(pos.x, pos.y), particleSize_, 1.0f);
  }
}

void Renderer::handleMouseReleased() {
  if (gSlider_.isDragging)
    gSlider_.isDragging = false;
  if (eSlider_.isDragging)
    eSlider_.isDragging = false;
  draggingAny_ = false;
}

void Renderer::handleMouseMoved(const sf::Event::MouseMoved &e) {
  if (!draggingAny_)
    return;
  if (gSlider_.isDragging)
    gSlider_.move(e.position);
  if (eSlider_.isDragging)
    eSlider_.move(e.position);
}

void Renderer::drawParticles() {
  for (auto &par : sim_.getParticles()) {
    Vec2f pos = par.position;
    const float r = par.radius;

    sf::CircleShape obj(r);
    obj.setOrigin({r, r});
    obj.setPosition({pos.x, pos.y});
    obj.setTexture(&particleTexture_);
    window_.draw(obj);
  }
}

void Renderer::drawComponents() {
  gSlider_.draw(window_);
  eSlider_.draw(window_);
  window_.draw(fpsText_);
  window_.draw(particleCountText_);
}

void Renderer::updateText() {
  sf::Time elapsed = frameClock_.restart();
  float fps = 1.0f / elapsed.asSeconds();
  fpsText_.setString("FPS: " + std::to_string(static_cast<int>(fps)));
  particleCountText_.setString("Particles: " +
                               std::to_string(sim_.getParticles().size()));
}

void Renderer::drawFrame() {
  updateText();
  window_.clear();
  drawParticles();
  drawComponents();
  window_.display();
}
