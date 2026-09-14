cmake_minimum_required(VERSION 3.28)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(rpe-tests LANGUAGES CXX C)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

  include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpe-physics"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpe-physics")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpe-render"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpe-render")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpengine"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpengine")
  enable_testing()
else()
  include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
endif()

rpe_ensure_googletest()

add_executable(rp_tests
  test_math_types.cpp
  test_transform.cpp
  test_rotation.cpp
  test_constraints2d.cpp
  test_shapes.cpp
  test_world_support.cpp
  test_mass_properties.cpp
  test_body_store.cpp
  test_dynamic_bvh.cpp
  test_pair_cache.cpp
  test_broadphase.cpp
  test_gjk.cpp
  test_epa.cpp
  test_narrowphase_regression.cpp
  test_manifold.cpp
  test_manifold_cache.cpp
  test_integrator.cpp
  test_friction.cpp
  test_solver.cpp
  test_conservation.cpp
  test_restitution.cpp
  test_world.cpp
  test_fixed_timestep.cpp
  test_renderable_store.cpp
  test_mesh_library.cpp
)

target_compile_features(rp_tests PRIVATE cxx_std_20)
target_compile_options(rp_tests PRIVATE -Wall -Wextra)
target_link_libraries(rp_tests PRIVATE
  GTest::gtest_main
  rpe-physics
  rpe-render
  rpengine
)

include(GoogleTest)
gtest_discover_tests(rp_tests EXTRA_ARGS --gtest_color=yes)
