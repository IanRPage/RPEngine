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

		for (Particle *p : data) {
			insert(p);
		}

		divided = true;
  };

	void query(std::vector<Particle *> &res, const sf::FloatRect &range) const {
		if (!boundary.findIntersection(range)) {
			return;
		}
		for (Particle *p : data) {
			if (range.contains(p->getCenter())) {
				res.push_back(p);
			}
		}
		if (divided) {
			ul->query(res, range);
			ur->query(res, range);
			bl->query(res, range);
			br->query(res, range);
		}
	};

public:
  QuadTree(sf::FloatRect bounds, size_t cap = 4)
      : capacity(cap), boundary(bounds), divided(false) {};

  void insert(Particle *p) {
		if (!boundary.contains(p->getCenter())) {
			return;
		}

		if (data.size() < capacity && !divided) {
			data.push_back(p);
			return;
		}

		if (!divided) {
			subdivide();
		}

		ul->insert(p);
		ur->insert(p);
		bl->insert(p);
		br->insert(p);
	};

  std::vector<Particle *> query(const sf::FloatRect &range) const {
		std::vector<Particle *> result;
		query(result, range);
		return result;
	};
};

#endif
