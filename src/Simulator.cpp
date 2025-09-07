#include <Simulator.h>

Simulator::Simulator(sf::Vector2u dims, float g, float C_r, float dt)
    : windowDims(dims), dt(dt), gravity(g), restitution(C_r) {
  std::random_device rd;
  gen.seed(rd());
};

void Simulator::wallCollisions() {
  auto [w, h] = windowDims;
  for (Particle &par : particles) {
    const float r = par.radius;

    // top/bot
    if (par.position.y < r) {
      par.position.y = r;
      par.velocity.y = -par.velocity.y * restitution;
    } else if (par.position.y > h - r) {
      par.position.y = h - r;
      par.velocity.y = -par.velocity.y * restitution;
    }

    // left/right
    if (par.position.x < r) {
      par.position.x = r;
      par.velocity.x = -par.velocity.x * restitution;
    } else if (par.position.x > w - r) {
      par.position.x = w - r;
      par.velocity.x = -par.velocity.x * restitution;
    }
  }
}

void Simulator::particleCollision(Particle &p1, Particle &p2) {
  const sf::Vector2f d = p2.position - p1.position;
  const float d2 = d.dot(d);
  const float sum_r = p1.radius + p2.radius;
  const float sum_r2 = sum_r * sum_r;

  if (d2 >= sum_r2 || d2 == 0.0f)
    return;

  const float invDist = 1.0f / std::sqrt(d2);
  const float dist = 1.0f / invDist;
  const sf::Vector2f norm = d * invDist;
  const float penetration = sum_r - dist;
  const float invMassSum = 1.0f / p1.mass + 1.0 / p2.mass;

  if (penetration > 0.0f && invMassSum > 0.0f) {
    const sf::Vector2f correction = norm * (penetration / invMassSum);
    p1.position -= correction * (1.0f / p1.mass);
    p2.position += correction * (1.0f / p2.mass);
  }

  const float relVel = (p2.velocity - p1.velocity).dot(norm);
  if (relVel < 0) {
    const float magJ = (1.0f + restitution) * relVel / invMassSum;
    const sf::Vector2f J = magJ * norm;
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
  QuadTree qtree(sf::FloatRect({0.0f, 0.0f}, {windowDims.x, windowDims.y}), 4);
  for (Particle &p : particles) {
    qtree.insert(&p);
  }

  for (size_t i = 0; i < particles.size(); i++) {
    Particle &p1 = particles[i];
    const sf::Vector2f c1 = p1.position;
    const float r1 = p1.radius;
    const sf::FloatRect queryRange({c1.x - 2.0f * r1, c1.y - 2.0f * r1},
                                   {4.0f * r1, 4.0f * r1});

    std::vector<Particle *> neighbors;
    qtree.query(neighbors, queryRange);

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
  // naiveCollisions();
  qtreeCollisions();
}

void Simulator::spawnParticle(sf::Vector2i pos, sf::Texture *texture) {
  std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
  particles.emplace_back(static_cast<sf::Vector2f>(pos),
                         sf::Vector2f{dist(gen), dist(gen)}, texture);
};

void Simulator::update() {
  for (Particle &par : particles) {
    par.acceleration = {0.0f, 0.0f};
    par.accelerate({0.0f, gravity});
    par.update(dt);
  }
  resolveCollisions();
}
