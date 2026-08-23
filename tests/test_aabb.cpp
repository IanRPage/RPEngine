#include <dsa/AABB.hpp>
#include <gtest/gtest.h>

TEST(AABBTest, Construction) {
  AABBf box({0.0f, 0.0f}, {10.0f, 20.0f});
  EXPECT_FLOAT_EQ(box.min.x, 0.0f);
  EXPECT_FLOAT_EQ(box.min.y, 0.0f);
  EXPECT_FLOAT_EQ(box.max.x, 10.0f);
  EXPECT_FLOAT_EQ(box.max.y, 20.0f);
}

TEST(AABBTest, WidthHeight) {
  AABBf box({5.0f, 5.0f}, {10.0f, 20.0f});
  EXPECT_FLOAT_EQ(box.width(), 10.0f);
  EXPECT_FLOAT_EQ(box.height(), 20.0f);
}

TEST(AABBTest, ContainsPointInside) {
  AABBf box({0.0f, 0.0f}, {10.0f, 10.0f});
  EXPECT_TRUE(box.contains({5.0f, 5.0f}));
}

TEST(AABBTest, ContainsPointOnBoundary) {
  AABBf box({0.0f, 0.0f}, {10.0f, 10.0f});
  EXPECT_TRUE(box.contains({0.0f, 0.0f}));
  EXPECT_TRUE(box.contains({10.0f, 10.0f}));
}

TEST(AABBTest, DoesNotContainPointOutside) {
  AABBf box({0.0f, 0.0f}, {10.0f, 10.0f});
  EXPECT_FALSE(box.contains({11.0f, 5.0f}));
  EXPECT_FALSE(box.contains({-1.0f, 5.0f}));
}

TEST(AABBTest, IntersectsOverlapping) {
  AABBf a({0.0f, 0.0f}, {10.0f, 10.0f});
  AABBf b({5.0f, 5.0f}, {10.0f, 10.0f});
  EXPECT_TRUE(a.intersects(b));
  EXPECT_TRUE(b.intersects(a));
}

TEST(AABBTest, IntersectsTouching) {
  AABBf a({0.0f, 0.0f}, {10.0f, 10.0f});
  AABBf b({10.0f, 0.0f}, {10.0f, 10.0f});
  EXPECT_TRUE(a.intersects(b));
}

TEST(AABBTest, DoesNotIntersectSeparated) {
  AABBf a({0.0f, 0.0f}, {10.0f, 10.0f});
  AABBf b({20.0f, 20.0f}, {10.0f, 10.0f});
  EXPECT_FALSE(a.intersects(b));
}
