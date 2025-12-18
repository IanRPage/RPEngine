#include <imgui.h>

#include <ui/Renderer.hpp>

#include "Simulator.hpp"

Renderer::Renderer(Simulator& sim, const Options& opts)
    : sim_{sim},
      window_{
          sf::RenderWindow(sf::VideoMode(sf::VideoMode::getDesktopMode().size),
                           opts.window_title)},
      imguiCtrl_{sim} {
  window_.setFramerateLimit(opts.fps_limit);

  // --- ImGui setup(-ish) ---
  if (!ImGui::SFML::Init(window_)) {
    throw std::runtime_error("Failed to initialize ImGui");
  }
  ImGui::GetIO().IniFilename = "build/imgui.ini";
  // -------------------------

  lastSize_ = window_.getSize();
  sim_.configure(
      {static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)},
      1.0f / static_cast<float>(opts.fps_limit));

  gen_ = std::mt19937(std::random_device{}());
  distX = std::uniform_real_distribution<float>(0.0f, lastSize_.x - 20.0f);
  distY = std::uniform_real_distribution<float>(0.0f, lastSize_.y - 20.0f);

  // set vertex array things
  particleVertices_.setPrimitiveType(sf::PrimitiveType::Triangles);
  computeUnitCircle();

  runtimeClock_.start();

  colorLUT_.assign(sim_.capacity(), std::optional<sf::Color>());

  layoutUI();
}

void Renderer::pollAndHandleEvents() noexcept {
  while (const std::optional event = window_.pollEvent()) {
    ImGui::SFML::ProcessEvent(window_, *event);
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
  spawn();
  radialPush();

  ImGui::SFML::Update(window_, frameClock_.restart());
  imguiCtrl_.render();
  window_.clear();
  drawParticles();
  ImGui::SFML::Render(window_);
  window_.display();
}

void Renderer::mainLoop() {
  while (isOpen()) {
    pollAndHandleEvents();
    sim_.update();
    drawFrame();
  }
  ImGui::SFML::Shutdown();
}

void Renderer::computeUnitCircle() {
  for (size_t s = MIN_CIRCLE_SEGMENTS; s <= MAX_CIRCLE_SEGMENTS; s++) {
    unitCircle_[s].clear();
    unitCircle_[s].reserve(s * 3);
    for (size_t k = 0; k < s; k++) {
      const float theta1 = 2.0f * PI * k / s;
      const float theta2 = 2.0f * PI * (k + 1) / s;

      unitCircle_[s].emplace_back(0.0f, 0.0f);
      unitCircle_[s].emplace_back(cos(theta1), sin(theta1));
      unitCircle_[s].emplace_back(cos(theta2), sin(theta2));
    }
  }
}

size_t Renderer::getCircleSegments(float radius) {
  const size_t segments = radius * 1.5f + MIN_CIRCLE_SEGMENTS;
  return std::clamp(segments, MIN_CIRCLE_SEGMENTS, MAX_CIRCLE_SEGMENTS);
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
  if (!colorLUT_[p.id]) {
    const float t = runtimeClock_.getElapsedTime().asSeconds();
    colorLUT_[p.id] = getRainbow(t);
  }
  return *colorLUT_[p.id];
}

void Renderer::layoutUI() noexcept {
  const auto size = window_.getSize();
  lastSize_ = size;
  sim_.setWorldSize(
      Vec2f(static_cast<float>(lastSize_.x), static_cast<float>(lastSize_.y)));
}

void Renderer::handleMousePressed(
    const sf::Event::MouseButtonPressed& e) noexcept {
  if (ImGui::GetIO().WantCaptureMouse) return;

  const auto m = window_.mapPixelToCoords(e.position, window_.getDefaultView());

  if (e.button == sf::Mouse::Button::Left) {
    if (sim_.forceType() == ForceType::Radial) {
      radialPushing_ = true;
      pushOrigin_ = m;
    }  // TODO: add more forces
  } else if (e.button == sf::Mouse::Button::Right) {
    sim_.spawnParticle({m.x, m.y}, {0.0f, 0.0f}, imguiCtrl_.particleRadius(),
                       1.0f);
  }
}

void Renderer::handleMouseReleased() noexcept {
  radialPushing_ = false;
}

void Renderer::handleMouseMoved(const sf::Event::MouseMoved& e) noexcept {
  // MAYBE: add mouse guard on ImGui panel
  if (ImGui::GetIO().WantCaptureMouse) return;
  const auto m = window_.mapPixelToCoords(e.position, window_.getDefaultView());
  if (radialPushing_) {
    pushOrigin_ = m;
  }
}

void Renderer::handleKeyPressed(const sf::Event::KeyPressed& e) noexcept {
  if (e.scancode == sf::Keyboard::Scan::Space) {
    if (sim_.spawnType() == imguiCtrl_.spawnType()) {
      sim_.setSpawnType(SpawnType::None);
    } else {
      sim_.setSpawnType(imguiCtrl_.spawnType());
    }
  }
}

void Renderer::drawParticles() {
  size_t totalVertexCount = 0;
  for (const Particle& par : sim_.particles()) {
    const size_t segments = getCircleSegments(par.radius);
    totalVertexCount += segments * 3;
  }

  if (particleVertices_.getVertexCount() < totalVertexCount) {
    particleVertices_.resize(totalVertexCount);
  }

  size_t vertexIdx = 0;
  for (const Particle& par : sim_.particles()) {
    const Vec2f& pos = par.position;
    const sf::Color& color = colorFor(par);
    const size_t segments = getCircleSegments(par.radius);

    // transform unit circle vertices
    const std::vector<sf::Vector2f>& vertices = unitCircle_[segments];
    for (size_t i = 0; i < vertices.size(); i++) {
      particleVertices_[vertexIdx] =
          sf::Vertex{sf::Vector2f(pos.x + par.radius * vertices[i].x,
                                  pos.y + par.radius * vertices[i].y),
                     color};
      vertexIdx++;
    }
  }
  window_.draw(particleVertices_);
}

void Renderer::spawn() noexcept {
  if (sim_.particles().size() >= sim_.capacity()) return;

  switch (sim_.spawnType()) {
    case SpawnType::None: return;

    case SpawnType::Random: {
      if (spawnClock_.getElapsedTime().asSeconds() >= spawnInterval_) {
        for (int i = 0; i < 5; i++) { // TODO: expose speed in panel
          sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f},
                             imguiCtrl_.particleRadius(), 1.0f);
        }
        spawnClock_.restart();
      }
      break;
    }

    case SpawnType::Stream: {
      if (spawnClock_.getElapsedTime().asSeconds() >= spawnInterval_) {
        const float speed = 1200.0f;  // tune these
        const float omega = 0.5f;     // parameters
        const float t = runtimeClock_.getElapsedTime().asSeconds();
        const float angle = 0.5f * PI * (cos(t * omega) + 1.0f);
        sim_.spawnParticle({lastSize_.x * 0.5f, 25.0f},
                           Vec2f(cos(angle), sin(angle)) * speed,
                           imguiCtrl_.particleRadius(), 1.0f);
        spawnClock_.restart();
      }
      break;
    }

    case SpawnType::Max: {
      const float baseTime = runtimeClock_.getElapsedTime().asSeconds();
      const size_t startIdx = sim_.particles().size();
      for (size_t i = startIdx; i < sim_.capacity(); i++) {
        sim_.spawnParticle({distX(gen_), distY(gen_)}, {0.0f, 0.0f},
                           imguiCtrl_.particleRadius(), 1.0f);
        const float t = baseTime + i * 0.001f;
        colorLUT_[i] = getRainbow(t);
      }
      sim_.setSpawnType(SpawnType::None);
      break;
    }
  }
}

void Renderer::radialPush() {
  if (!radialPushing_) return;

  const float pDiam = imguiCtrl_.radialPushRadius() * 2;

  sim_.radialPush({pushOrigin_.x, pushOrigin_.y}, pDiam,
                  imguiCtrl_.forceMagnitude());
}
