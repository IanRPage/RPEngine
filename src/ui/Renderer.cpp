#include <ui/Renderer.hpp>

Renderer::Renderer(Simulator& sim, const Options& opts)
    : sim_(sim),
      window_(
          sf::RenderWindow(sf::VideoMode(sf::VideoMode::getDesktopMode().size),
                           opts.window_title)),
      gSlider_({40.0f, 0.0f}, {200.0f, 10.0f}, {-100.0f, 100.0f}, sim_.gravity,
               font_, "Gravity"),
      eSlider_({40.0f, 0.0f}, {200.0f, 10.0f}, {0.0f, 1.0f}, sim_.restitution,
               font_, "Restitution", sf::Color::White, sf::Color::Yellow),
      particleCountText_(font_, "Particles: ", 30),
      fpsText_(font_, "FPS: 60", 30) {
  window_.setFramerateLimit(opts.fps_limit);
  lastSize_ = window_.getSize();
  sim_.configure(
      {static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)},
      1.0f / static_cast<float>(opts.fps_limit));

  gen_ = std::mt19937(std::random_device{}());
  distX = std::uniform_real_distribution<float>(0.0f, lastSize_.x - 20.0f);
  distY = std::uniform_real_distribution<float>(0.0f, lastSize_.y - 20.0f);

  // load font
  if (!font_.openFromFile("assets/pixelated.ttf")) {
    throw std::runtime_error("Failed to load font: assets/pixelated.ttf");
  };

  particleSize_ = sim.maxParticleRadius();
  particleShape_.setRadius(particleSize_);
  particleShape_.setOrigin({particleSize_, particleSize_});

  // initialize fps measurement arrays
  frameTimes_.fill(1.0f / static_cast<float>(opts.fps_limit));

  runtimeClock_.start();

  layoutUI();
}

void Renderer::pollAndHandleEvents() noexcept {
  while (const std::optional event = window_.pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
      window_.close();
    } else if (const auto* mousePressed =
                   event->getIf<sf::Event::MouseButtonPressed>()) {
      handleMousePressed(*mousePressed);
    } else if (event->is<sf::Event::MouseButtonReleased>()) {
      handleMouseReleased();
    } else if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
      handleMouseMoved(*mouseMoved);
    } else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
      handleKeyPressed(*keyPressed);
    }
  }
}

void Renderer::drawFrame() {
  updateText();
  randomSpawn();
  streamSpawn();
  randomSpawnSUPERFAST();
  spawnMax();
  window_.clear();
  drawParticles();
  drawComponents();
  window_.display();
}

const sf::Color Renderer::getRainbow(float t) noexcept {
  const float r = sin(t);
  const float g = sin(t + 0.33f * 2.0f * PI);
  const float b = sin(t + 0.66f * 2.0f * PI);
  return {static_cast<uint8_t>(255.0f * r * r),
          static_cast<uint8_t>(255.0f * g * g),
          static_cast<uint8_t>(255.0f * b * b)};
}

const sf::Color& Renderer::colorFor(const Particle& p) noexcept {
  auto it = colorLUT_.find(p.id);

  if (it != colorLUT_.end())  // if color is found
    return it->second;
  const float t = runtimeClock_.getElapsedTime().asSeconds();
  auto [inserted, _] = colorLUT_.emplace(p.id, getRainbow(t));
  return inserted->second;
}

void Renderer::layoutUI() noexcept {
  const auto size = window_.getSize();
  lastSize_ = size;
  sim_.setWorldSize(
      Vec2f(static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)));

  const float margin = 10.0f;
  const auto [sliderWidth, sliderHeight] = gSlider_.getSize();

  // button heights
  const float y1 = static_cast<float>(size.y) - 100.0f;
  const float y2 = static_cast<float>(size.y) - 50.0f;

  gSlider_.setPosition({margin, y1});
  gSlider_.setSize({sliderWidth, sliderHeight});

  eSlider_.setPosition({margin, y2});
  eSlider_.setSize({sliderWidth, sliderHeight});

  // top corner text
  particleCountText_.setPosition({margin, margin});
  float fpsTextWidth = fpsText_.getLocalBounds().size.x;
  fpsText_.setPosition(
      {static_cast<float>(size.x) - fpsTextWidth - margin, margin});
}

void Renderer::handleMousePressed(
    const sf::Event::MouseButtonPressed& e) noexcept {
  if (e.button != sf::Mouse::Button::Left) return;

  const auto m = window_.mapPixelToCoords(e.position, window_.getDefaultView());
  if (gSlider_.contains(m)) {
    gSlider_.isDragging = true;
    gSlider_.setActive(true);
    draggingAny_ = true;
  } else if (eSlider_.contains(m)) {
    eSlider_.isDragging = true;
    eSlider_.setActive(true);
    draggingAny_ = true;
  } else {
    sim_.spawnParticle({m.x, m.y}, {0.0f, 0.0f}, particleSize_, 1.0f);
  }
}

void Renderer::handleMouseReleased() noexcept {
  if (gSlider_.isDragging) {
    gSlider_.isDragging = false;
    gSlider_.setActive(false);
  } else if (eSlider_.isDragging) {
    eSlider_.isDragging = false;
    eSlider_.setActive(false);
  }
  draggingAny_ = false;
}

void Renderer::handleMouseMoved(const sf::Event::MouseMoved& e) noexcept {
  if (!draggingAny_) return;

  const auto m = window_.mapPixelToCoords(e.position, window_.getDefaultView());
  if (gSlider_.isDragging) gSlider_.move(m);
  if (eSlider_.isDragging) eSlider_.move(m);
}

void Renderer::handleKeyPressed(const sf::Event::KeyPressed& e) noexcept {
  if (e.scancode == sf::Keyboard::Scan::R) {
    randomSpawn_ = !randomSpawn_;
    streamSpawn_ = false;
    randomSpawnSUPERFAST_ = false;
    spawnMax_ = false;
  } else if (e.scancode == sf::Keyboard::Scan::Space) {
    streamSpawn_ = !streamSpawn_;
    randomSpawn_ = false;
    spawnMax_ = false;
    randomSpawnSUPERFAST_ = false;
  } else if (e.scancode == sf::Keyboard::Scan::F) {
    randomSpawnSUPERFAST_ = !randomSpawnSUPERFAST_;
    randomSpawn_ = false;
    streamSpawn_ = false;
    spawnMax_ = false;
  } else if (e.scancode == sf::Keyboard::Scan::M) {
    spawnMax_ = !spawnMax_;
    randomSpawn_ = false;
    randomSpawnSUPERFAST_ = false;
    streamSpawn_ = false;
  }
}

void Renderer::drawParticles() {
  for (auto& par : sim_.particles()) {
    Vec2f pos = par.position;
    particleShape_.setPosition({pos.x, pos.y});
    particleShape_.setFillColor(colorFor(par));
    window_.draw(particleShape_);
  }
}

void Renderer::drawComponents() {
  gSlider_.draw(window_);
  eSlider_.draw(window_);
  window_.draw(fpsText_);
  window_.draw(particleCountText_);
}

void Renderer::updateText() noexcept {
  sf::Time elapsed = frameClock_.restart();
  float frameTime = elapsed.asSeconds();

  // store frame time in circular buffer
  frameTimes_[frameIndex_] = frameTime;
  frameIndex_ = (frameIndex_ + 1) % FPS_SAMPLE_COUNT;

  // mark as collected after first full cycle
  if (frameIndex_ == 0) samplesCollected_ = true;

  // Calculate average FPS only after we have enough samples
  if (samplesCollected_) {
    float avgFrameTime =
        std::accumulate(frameTimes_.begin(), frameTimes_.end(), 0.0f) /
        FPS_SAMPLE_COUNT;
    float fps = 1.0f / avgFrameTime;
    fpsText_.setString("FPS: " + std::to_string(static_cast<int>(fps)));
  }

  particleCountText_.setString("Particles: " +
                               std::to_string(sim_.particles().size()));
}

void Renderer::randomSpawn() noexcept {
  if (!randomSpawn_ || sim_.particles().size() >= sim_.capacity()) return;

  if (spawnClock_.getElapsedTime().asSeconds() >= spawnInterval_) {
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    spawnClock_.restart();
  }
}

void Renderer::randomSpawnSUPERFAST() noexcept {
  if (!randomSpawnSUPERFAST_ || sim_.particles().size() >= sim_.capacity())
    return;

  if (spawnClock_.getElapsedTime().asSeconds() >= spawnInterval_) {
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    spawnClock_.restart();
  }
}

void Renderer::streamSpawn() noexcept {
  if (!streamSpawn_ || sim_.particles().size() >= sim_.capacity()) return;

  if (spawnClock_.getElapsedTime().asSeconds() >= spawnInterval_) {
    const float speed = 1200.0f;  // tune these
    const float omega = 0.5f;     // parameters
    const float t = runtimeClock_.getElapsedTime().asSeconds();
    const float angle = 0.5f * PI * (cos(t * omega) + 1.0f);
    sim_.spawnParticle({lastSize_.x * 0.5f, 25.0f},
                       Vec2f(cos(angle), sin(angle)) * speed, particleSize_,
                       1.0f);
    spawnClock_.restart();
  }
}

void Renderer::spawnMax() noexcept {
  if (!spawnMax_ || sim_.particles().size() >= sim_.capacity()) return;

  const float baseTime = runtimeClock_.getElapsedTime().asSeconds();
  for (size_t i = 0; i < sim_.capacity(); i++) {
    sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f}, particleSize_,
                       1.0f);
    const float t = baseTime + i * 0.001f;
    colorLUT_.emplace(static_cast<uint32_t>(i), getRainbow(t));
  }
  spawnMax_ = false;
}
