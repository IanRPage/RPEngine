#include <gtest/gtest.h>
#include <render/RenderableStore.hpp>

TEST(RenderableStoreTest, UnassignedHandleReturnsDefaultColor) {
  render::RenderableStore store;
  BodyHandle handle{0, 0};

  EXPECT_FALSE(store.hasColor(handle));
  EXPECT_EQ(store.colorOr(handle), render::RenderableStore::kDefaultColor);
}

TEST(RenderableStoreTest, SetColorRoundTrips) {
  render::RenderableStore store;
  BodyHandle handle{3, 1};
  Vec4f red{1.0f, 0.0f, 0.0f, 1.0f};

  store.setColor(handle, red);

  EXPECT_TRUE(store.hasColor(handle));
  EXPECT_EQ(store.colorOr(handle), red);
}

TEST(RenderableStoreTest, StaleGenerationFallsBackAfterSlotReuse) {
  render::RenderableStore store;
  BodyHandle original{5, 0};
  BodyHandle reused{5, 1};
  Vec4f red{1.0f, 0.0f, 0.0f, 1.0f};

  store.setColor(original, red);

  EXPECT_FALSE(store.hasColor(reused));
  EXPECT_EQ(store.colorOr(reused), render::RenderableStore::kDefaultColor);
  EXPECT_EQ(store.colorOr(reused, Vec4f(0.0f)), Vec4f(0.0f));
}

TEST(RenderableStoreTest, ClearRemovesAllEntries) {
  render::RenderableStore store;
  BodyHandle handle{0, 0};
  store.setColor(handle, Vec4f(1.0f, 1.0f, 1.0f, 1.0f));

  store.clear();

  EXPECT_FALSE(store.hasColor(handle));
}
