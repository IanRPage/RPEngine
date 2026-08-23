#ifndef CONTACT_HPP
#define CONTACT_HPP

#include <cstddef>
#include <dsa/Vec2.hpp>

struct Contact {
  size_t indexA;
  size_t indexB;
  Vec2f normal;
  float penetration;

  Contact(size_t idxA, size_t idxB, Vec2f n, float p)
      : indexA(idxA), indexB(idxB), normal(n), penetration(p) {}
};

#endif
