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

void Simulator::particleCollision(Particle &p1, Particle &p2) {
  sf::Vector2f c1 = p1.position + sf::Vector2f(p1.radius, p1.radius);
  sf::Vector2f c2 = p2.position + sf::Vector2f(p2.radius, p2.radius);
  float dist = (c2 - c1).length();
  float sum_r = p1.radius + p2.radius;
  if (dist > sum_r)
    return;

  float massInverse = 1.0f / p1.mass + 1.0 / p2.mass;
  sf::Vector2f norm = (c2 - c1) / dist;

  // overlap correction using each particle's inertia
  float penetration = sum_r - dist;
  if (penetration > 0) {
    sf::Vector2f correction = norm * (penetration / massInverse);
    p1.position -= correction * (1.0f / p1.mass);
    p2.position += correction * (1.0f / p2.mass);
  }

  float relVel = (p2.velocity - p1.velocity).dot(norm);
  if (relVel < 0) {
    float magJ = (1.0f + restitution) * relVel / massInverse;
    sf::Vector2f J = magJ * norm;
    p1.velocity += J / p1.mass;
    p2.velocity -= J / p2.mass;
  }
}

// O(n^2)
void Simulator::naiveCollisions() {
  for (size_t i = 0; i < particles.size(); i++) {
    for (size_t j = i + 1; j < particles.size(); j++) {
      particleCollision(particles[i], particles[j]);
    }
  }
}

// O(nlog(n))
void Simulator::qtreeCollisions() {
	float radialDist = 2.0f * 15.0f;
  QuadTree qtree(sf::FloatRect({0.0f, 0.0f}, {windowDims.x, windowDims.y}), 4);
  for (Particle &p : particles) {
    qtree.insert(&p);
  }

  for (size_t i = 0; i < particles.size(); i++) {
    Particle &p1 = particles[i];
    sf::Vector2f c1 = p1.getCenter();
    float r1 = p1.radius;
    sf::FloatRect queryRange({c1.x - radialDist, c1.y - radialDist},
                             {4.0f * r1, 4.0f * r1});

		std::vector<Particle *> neighbors = qtree.query(queryRange);

		for (Particle *nei : neighbors) {
			Particle &p2 = *nei;
			if (&p1 <= &p2) {
				continue;
			}
			particleCollision(p1, p2);
		}
  }
}

void Simulator::resolveCollisions() {
  wallCollisions();
	naiveCollisions();
	// qtreeCollisions();
}

void Simulator::spawnParticle(sf::Vector2i pos, sf::Texture *texture) {
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);

  Particle circle(static_cast<sf::Vector2f>(pos), {dist(gen), dist(gen)},
                  texture);
  particles.push_back(circle);
}

void Simulator::update() {
  for (Particle &par : particles) {
    par.acceleration = {0.0f, 0.0f};
    par.accelerate({0.0f, gravity}); // apply gravity
    par.update(dt);
  }
  resolveCollisions();
}
