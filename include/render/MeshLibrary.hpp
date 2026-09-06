#ifndef RPENGINE_RENDER_MESHLIBRARY_HPP
#define RPENGINE_RENDER_MESHLIBRARY_HPP

#include <collision/Shapes.hpp>
#include <render/Mesh.hpp>

namespace render {

MeshData buildUnitQuad() noexcept;
MeshData buildUnitCircle(int segments = 32) noexcept;
MeshData buildUnitIcosphere(int subdivisions = 2) noexcept;
MeshData buildUnitBox() noexcept;
MeshData buildUnitCapsule(int radialSegments = 16, int capRings = 8) noexcept;

MeshData triangulateConvexHull(const ConvexHullShape& hull) noexcept;

}  // namespace render

#endif
