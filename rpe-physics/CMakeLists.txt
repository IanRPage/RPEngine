cmake_minimum_required(VERSION 3.28)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(rpe-physics LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
rpe_ensure_glm()

add_library(rpe-physics STATIC
  src/math/Transform.cpp
  src/math/Rotation.cpp
  src/math/Constraints2D.cpp
  src/collision/Shapes.cpp
  src/collision/MassProperties.cpp
  src/collision/Gjk.cpp
  src/collision/Epa.cpp
  src/collision/ManifoldBuilder.cpp
  src/collision/ManifoldCache.cpp
  src/core/BodyStore.cpp
  src/core/World.cpp
  src/dynamics/Integrator.cpp
  src/dynamics/Friction.cpp
  src/dynamics/Solver.cpp
  src/broadphase/DynamicBVH.cpp
  src/broadphase/PairCache.cpp
  src/broadphase/NaiveBroadphase.cpp
  src/broadphase/GridBroadphase.cpp
  src/broadphase/DynamicBVHBroadphase.cpp
)
add_library(RPEngine::Physics ALIAS rpe-physics)

target_compile_features(rpe-physics PUBLIC cxx_std_20)
target_include_directories(rpe-physics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_options(rpe-physics PRIVATE
  -Wall
  -Wextra
  $<$<CONFIG:Debug>:-O0;-g>
  $<$<CONFIG:Release>:-O3>
)
target_link_libraries(rpe-physics PUBLIC glm::glm)
