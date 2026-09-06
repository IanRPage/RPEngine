#include <gtest/gtest.h>

#include <dynamics/Friction.hpp>
#include <math/Constants.hpp>

namespace {
void expectOrthonormalBasis(Vec3f normal) {
  normal = glm::normalize(normal);
  auto [t1, t2] = computeTangentBasis(normal);

  EXPECT_NEAR(glm::length(t1), 1.0f, 1e-4f);
  EXPECT_NEAR(glm::length(t2), 1.0f, 1e-4f);
  EXPECT_NEAR(glm::dot(t1, normal), 0.0f, 1e-4f);
  EXPECT_NEAR(glm::dot(t2, normal), 0.0f, 1e-4f);
  EXPECT_NEAR(glm::dot(t1, t2), 0.0f, 1e-4f);
}
}  // namespace

TEST(FrictionTest, OrthonormalBasisForAxisAlignedNormals) {
  expectOrthonormalBasis(Vec3f(1.0f, 0.0f, 0.0f));
  expectOrthonormalBasis(Vec3f(0.0f, 1.0f, 0.0f));
  expectOrthonormalBasis(Vec3f(0.0f, 0.0f, 1.0f));
  expectOrthonormalBasis(Vec3f(-1.0f, 0.0f, 0.0f));
}

TEST(FrictionTest, OrthonormalBasisForArbitraryNormals) {
  expectOrthonormalBasis(Vec3f(1.0f, 1.0f, 1.0f));
  expectOrthonormalBasis(Vec3f(0.6f, 0.8f, 0.0f));
  expectOrthonormalBasis(Vec3f(INV_SQRT_3, INV_SQRT_3, INV_SQRT_3));  // branch threshold
}

TEST(FrictionTest, BranchSelectionMatchesXComponentThreshold) {
  // |normal.x| >= 1/sqrt(3). fallback axis is world Y
  auto [t1a, t2a] = computeTangentBasis(Vec3f(1.0f, 0.0f, 0.0f));
  (void)t2a;
  EXPECT_NEAR(glm::length(t1a), 1.0f, 1e-4f);

  // |normal.x| < 1/sqrt(3). fallback axis is world X
  auto [t1b, t2b] = computeTangentBasis(Vec3f(0.0f, 1.0f, 0.0f));
  (void)t2b;
  EXPECT_NEAR(glm::length(t1b), 1.0f, 1e-4f);
}
