#include <collision/Gjk.hpp>

#include <cmath>
#include <math/Constants.hpp>

SupportPoint minkowskiSupport(const ShapeVariant& shapeA, const Transform& ta,
                              const ShapeVariant& shapeB, const Transform& tb,
                              Vec3f dir) noexcept {
  Vec3f pA = worldSupport(shapeA, ta, dir);
  Vec3f pB = worldSupport(shapeB, tb, -dir);
  return SupportPoint{pA, pB, pA - pB};
}

namespace {

Vec3f tripleCross(Vec3f a, Vec3f b, Vec3f c) noexcept {
  return glm::cross(glm::cross(a, b), c);
}

Vec3f arbitraryPerpendicular(Vec3f v) noexcept {
  Vec3f inPlane(-v.y, v.x, 0.0f);
  if (glm::dot(inPlane, inPlane) > VECTOR_LENGTH_EPSILON) return inPlane;
  return glm::cross(v, Vec3f(1.0f, 0.0f, 0.0f));
}

Vec3f perpendicularTowards(Vec3f edge, Vec3f towards) noexcept {
  Vec3f d = tripleCross(edge, towards, edge);
  if (glm::dot(d, d) < VECTOR_LENGTH_EPSILON) return arbitraryPerpendicular(edge);
  return d;
}

void setPoint(std::array<SupportPoint, 4>& s, const SupportPoint& a,
             int& count) noexcept {
  s[0] = a;
  count = 1;
}

void setLine(std::array<SupportPoint, 4>& s, const SupportPoint& a,
            const SupportPoint& b, int& count) noexcept {
  s[0] = a;
  s[1] = b;
  count = 2;
}

void setTriangle(std::array<SupportPoint, 4>& s, const SupportPoint& a,
                 const SupportPoint& b, const SupportPoint& c,
                 int& count) noexcept {
  s[0] = a;
  s[1] = b;
  s[2] = c;
  count = 3;
}

void lineCase(std::array<SupportPoint, 4>& s, int& count, Vec3f& dir) noexcept {
  const SupportPoint a = s[0];
  const SupportPoint b = s[1];
  const Vec3f ab = b.diff - a.diff;
  const Vec3f ao = -a.diff;

  if (glm::dot(ab, ao) > 0.0f) {
    setLine(s, a, b, count);
    dir = perpendicularTowards(ab, ao);
  } else {
    setPoint(s, a, count);
    dir = ao;
  }
}

void triangleCase(std::array<SupportPoint, 4>& s, int& count,
                  Vec3f& dir) noexcept {
  const SupportPoint a = s[0];
  const SupportPoint b = s[1];
  const SupportPoint c = s[2];
  const Vec3f ab = b.diff - a.diff;
  const Vec3f ac = c.diff - a.diff;
  const Vec3f ao = -a.diff;
  const Vec3f abc = glm::cross(ab, ac);

  if (glm::dot(glm::cross(abc, ac), ao) > 0.0f) {
    if (glm::dot(ac, ao) > 0.0f) {
      setLine(s, a, c, count);
      dir = perpendicularTowards(ac, ao);
    } else if (glm::dot(ab, ao) > 0.0f) {
      setLine(s, a, b, count);
      dir = perpendicularTowards(ab, ao);
    } else {
      setPoint(s, a, count);
      dir = ao;
    }
    return;
  }

  if (glm::dot(glm::cross(ab, abc), ao) > 0.0f) {
    if (glm::dot(ab, ao) > 0.0f) {
      setLine(s, a, b, count);
      dir = perpendicularTowards(ab, ao);
    } else {
      setPoint(s, a, count);
      dir = ao;
    }
    return;
  }

  if (glm::dot(abc, ao) > 0.0f) {
    setTriangle(s, a, b, c, count);
    dir = abc;
  } else {
    setTriangle(s, a, c, b, count);
    dir = -abc;
  }
}

bool tetrahedronCase(std::array<SupportPoint, 4>& s, int& count,
                     Vec3f& dir) noexcept {
  const SupportPoint a = s[0];
  const SupportPoint b = s[1];
  const SupportPoint c = s[2];
  const SupportPoint d = s[3];
  const Vec3f ab = b.diff - a.diff;
  const Vec3f ac = c.diff - a.diff;
  const Vec3f ad = d.diff - a.diff;
  const Vec3f ao = -a.diff;

  Vec3f abc = glm::cross(ab, ac);

  if (std::abs(glm::dot(abc, ad)) < VECTOR_LENGTH_EPSILON) {
    setTriangle(s, b, c, d, count);
    return true;
  }

  Vec3f acd = glm::cross(ac, ad);
  Vec3f adb = glm::cross(ad, ab);

  if (glm::dot(abc, ad) > 0.0f) abc = -abc;
  if (glm::dot(acd, ab) > 0.0f) acd = -acd;
  if (glm::dot(adb, ac) > 0.0f) adb = -adb;

  if (glm::dot(abc, ao) > 0.0f) {
    setTriangle(s, a, b, c, count);
    dir = abc;
    return false;
  }
  if (glm::dot(acd, ao) > 0.0f) {
    setTriangle(s, a, c, d, count);
    dir = acd;
    return false;
  }
  if (glm::dot(adb, ao) > 0.0f) {
    setTriangle(s, a, d, b, count);
    dir = adb;
    return false;
  }
  return true;
}

bool isDuplicate(const std::array<SupportPoint, 4>& s, int count,
                 const Vec3f& diff) noexcept {
  for (int i = 0; i < count; ++i) {
    Vec3f delta = s[i].diff - diff;
    if (glm::dot(delta, delta) < VECTOR_LENGTH_EPSILON) return true;
  }
  return false;
}

}  // namespace

GjkResult gjkOverlap(const ShapeVariant& shapeA, const Transform& ta,
                     const ShapeVariant& shapeB, const Transform& tb) noexcept {
  GjkResult result;

  Vec3f dir = tb.position - ta.position;
  if (glm::dot(dir, dir) < VECTOR_LENGTH_EPSILON) dir = Vec3f(1.0f, 0.0f, 0.0f);

  result.simplex[0] = minkowskiSupport(shapeA, ta, shapeB, tb, dir);
  result.simplexCount = 1;
  dir = -result.simplex[0].diff;
  if (glm::dot(dir, dir) < VECTOR_LENGTH_EPSILON) dir = Vec3f(1.0f, 0.0f, 0.0f);

  constexpr int kMaxIterations = 32;
  for (int iter = 0; iter < kMaxIterations; ++iter) {
    SupportPoint newPoint = minkowskiSupport(shapeA, ta, shapeB, tb, dir);
    if (glm::dot(newPoint.diff, dir) < 0.0f) {
      result.overlapping = false;
      return result;
    }
    if (result.simplexCount >= 3 &&
        isDuplicate(result.simplex, result.simplexCount, newPoint.diff)) {
      result.overlapping = true;
      return result;
    }

    for (int i = result.simplexCount; i > 0; --i) {
      result.simplex[i] = result.simplex[i - 1];
    }
    result.simplex[0] = newPoint;
    ++result.simplexCount;

    if (result.simplexCount == 4) {
      if (tetrahedronCase(result.simplex, result.simplexCount, dir)) {
        result.overlapping = true;
        return result;
      }
    } else if (result.simplexCount == 3) {
      triangleCase(result.simplex, result.simplexCount, dir);
    } else {
      lineCase(result.simplex, result.simplexCount, dir);
    }
  }

  result.overlapping = false;
  return result;
}
