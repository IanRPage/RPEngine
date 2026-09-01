#include <cmath>
#include <gtest/gtest.h>
#include <math/Rotation.hpp>

TEST(RotationTest, IntegrateOrientationMatchesAnalyticConstantSpin) {
  const float omega = 1.5f;        // rad/s, spin about world Z
  const float dt = 1.0f / 240.0f;
  const int steps = 240;
  Quatf q{1.0f, 0.0f, 0.0f, 0.0f};

  for (int i = 0; i < steps; i++) {
    q = integrateOrientation(q, Vec3f{0.0f, 0.0f, omega}, dt);
  }

  float theta = omega * steps * dt;  // total angle after 1 second
  Quatf expected{std::cos(theta * 0.5f), 0.0f, 0.0f, std::sin(theta * 0.5f)};

  // first-order integration accumulates some error over 240 steps (1e-3 is a
  // reasonable tolerance at this step count)
  EXPECT_NEAR(std::abs(glm::dot(q, expected)), 1.0f, 1e-3f);
}

TEST(RotationTest, NlerpTakesShortestPath) {
  Quatf a{1.0f, 0.0f, 0.0f, 0.0f};
  Quatf b = -Quatf{glm::cos(0.1f), 0.0f, 0.0f, glm::sin(0.1f)};
  ASSERT_LT(glm::dot(a, b), 0.0f);

  Quatf result = nlerp(a, b, 0.5f);
  // after correcting for sign flip, the result should be much closer to the
  // small positive rotation than to a large one
  Quatf shortPathMidpoint = glm::normalize(a + Quatf{glm::cos(0.05f), 0.0f, 0.0f, glm::sin(0.05f)});
  EXPECT_GT(glm::abs(glm::dot(result, shortPathMidpoint)), 0.99f);
}
