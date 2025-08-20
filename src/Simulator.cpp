#include <Simulator.h>

Simulator::Simulator(sf::Vector2u dims, float g, float C_r, float dt)
    : windowDims(dims), dt(dt), gravity(g), restitution(C_r) {
  std::random_device rd;
  gen.seed(rd());
};

void Simulator::spawnParticle(sf::Vector2i pos) {
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);

  Particle circle(static_cast<sf::Vector2f>(pos), {dist(gen), dist(gen)});
  particles.push_back(circle);
}

void Simulator::update() {
  for (Particle &par : particles) {
    par.accelerate({0.0f, gravity}); // apply gravity
    par.update(dt);
  }
	handleCollisions();
}

void Simulator::handleCollisions() {
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
