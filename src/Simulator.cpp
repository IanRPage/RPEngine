#include <Simulator.h>

Simulator::Simulator(sf::Vector2u dims, float g, float C_r, float dt)
    : windowDims(dims), dt(dt), gravity(g), restitution(C_r) {
  std::random_device rd;
  gen.seed(rd());
};

void Simulator::wallCollisions() {
  auto [w, h] = windowDims;
  for (Particle &par : particles) {
    float r = par.radius;

    // top
    if (par.position.y < 0) {
      par.position.y = 0;
      par.velocity.y = -par.velocity.y * restitution;
    }

    // bot
    if (par.position.y + 2 * r > h) {
      par.position.y = h - 2 * r;
      par.velocity.y = -par.velocity.y * restitution;
    }

    // left
    if (par.position.x < 0) {
      par.position.x = 0;
      par.velocity.x = -par.velocity.x * restitution;
    }

    // right
    if (par.position.x + 2 * r > w) {
      par.position.x = w - 2 * r;
      par.velocity.x = -par.velocity.x * restitution;
    }

    par.shape.setPosition(par.position);
  }
}

void Simulator::particleCollisions() {
  for (size_t i = 0; i < particles.size(); i++) {
    for (size_t j = 0; j < particles.size() && j != i; j++) {
      Particle &p1 = particles[i], &p2 = particles[j];
      sf::Vector2f c1 = p1.position + sf::Vector2f(p1.radius, p1.radius);
      sf::Vector2f c2 = p2.position + sf::Vector2f(p2.radius, p2.radius);
			float dist = (c2 - c1).length();
			if (dist < p1.radius + p2.radius) {
				sf::Vector2f norm = (c2 - c1) / dist;
				float relVel = (p2.velocity - p1.velocity).dot(norm);
				if (relVel < 0) {
					sf::Vector2f impulse = norm * (relVel * (1 + restitution));
					p1.velocity += impulse;
					p2.velocity -= impulse;
				}
			}
    }
  }
}

void Simulator::handleCollisions() {
  wallCollisions();
  particleCollisions();
}

void Simulator::spawnParticle(sf::Vector2i pos) {
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);

  Particle circle(static_cast<sf::Vector2f>(pos), {dist(gen), dist(gen)});
  particles.push_back(circle);
}

void Simulator::update() {
  for (Particle &par : particles) {
    par.acceleration = {0.0f, 0.0f};
    par.accelerate({0.0f, gravity}); // apply gravity
    par.update(dt);
  }
  handleCollisions();
}
