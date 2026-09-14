#include <gtest/gtest.h>
#include <render/MeshLibrary.hpp>

TEST(MeshLibraryTest, UnitQuadHasExpectedTopology) {
  render::MeshData mesh = render::buildUnitQuad();

  EXPECT_EQ(mesh.vertices.size(), 4u);
  EXPECT_EQ(mesh.indices.size(), 6u);
}

TEST(MeshLibraryTest, UnitBoxHasSixIndependentFaces) {
  render::MeshData mesh = render::buildUnitBox();

  EXPECT_EQ(mesh.vertices.size(), 24u);
  EXPECT_EQ(mesh.indices.size(), 36u);
}
