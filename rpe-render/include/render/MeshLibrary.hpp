#ifndef RPENGINE_RENDER_MESHLIBRARY_HPP
#define RPENGINE_RENDER_MESHLIBRARY_HPP

#include <render/Mesh.hpp>

namespace render {

MeshData buildUnitQuad();
MeshData buildUnitCircle(int segments = 32);
MeshData buildUnitIcosphere(int subdivisions = 2);
MeshData buildUnitBox();
MeshData buildUnitCapsule(int radialSegments = 16, int capRings = 8);

MeshData buildUnitStadium(int capSegments = 16);

}  // namespace render

#endif
