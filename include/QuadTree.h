#ifndef QUADTREE_H
#define QUADTREE_H

#include <Particle.h>
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

class QuadTree {
private:
  size_t capacity;
  std::vector<Particle *> data;
  sf::FloatRect boundary;
  bool divided;

  std::unique_ptr<QuadTree> ul;
  std::unique_ptr<QuadTree> ur;
  std::unique_ptr<QuadTree> bl;
  std::unique_ptr<QuadTree> br;

  void subdivide() {
		auto [x, y] = boundary.position;
		auto [w, h] = boundary.size / 2.0f;

		ul = std::make_unique<QuadTree>(sf::FloatRect({x, y}, {w, h}), capacity);
		ur = std::make_unique<QuadTree>(sf::FloatRect({x + w, y}, {w, h}), capacity);
		bl = std::make_unique<QuadTree>(sf::FloatRect({x, y + h}, {w, h}), capacity);
		br = std::make_unique<QuadTree>(sf::FloatRect({x + w, y + h}, {w, h}), capacity);

		divided = true;
		for (Particle *p : data) {
			insert(p);
		}
		data.clear();
	};

public:
  QuadTree(sf::FloatRect bound, size_t cap)
      : capacity(cap), boundary(bound), divided(false) {};

  bool insert(Particle *p) {
		if (!boundary.contains(p->getCenter())) {
			return false;
		}

		if (data.size() < capacity && !divided) {
			data.push_back(p);
			return true;
		}

		if (!divided) {
			subdivide();
		}

		if (ul->insert(p)) return true;
		if (ur->insert(p)) return true;
		if (bl->insert(p)) return true;
		if (br->insert(p)) return true;
		return false;
	};

  void query(std::vector<Particle *> &res, const sf::FloatRect &qRange) const {
		if (!boundary.findIntersection(qRange)) {
			return;
		}

		for (Particle *p : data) {
			if (qRange.contains(p->getCenter())) {
				res.push_back(p);
			}
		}

		if (divided) {
			ul->query(res, qRange);
			ur->query(res, qRange);
			bl->query(res, qRange);
			br->query(res, qRange);
		}
  };

  std::vector<Particle *> query(const sf::FloatRect &qRange) const {
		std::vector<Particle *> result;
		query(result, qRange);
		return result;
	};
};

#endif
