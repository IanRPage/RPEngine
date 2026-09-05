include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(googletest)

file(GLOB_RECURSE ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "main\\.cpp$")
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "^${CMAKE_SOURCE_DIR}/src/ui/")
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "^${CMAKE_SOURCE_DIR}/src/render/")

add_executable(rp_tests
    test_vec2.cpp
    test_aabb.cpp
    test_particle.cpp
    test_spatial_grid.cpp
    test_simulator.cpp
    test_iterative_solver.cpp
    test_constraints.cpp
    test_collision_math.cpp
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
    ${ENGINE_SOURCES}
)

target_include_directories(rp_tests PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_features(rp_tests PRIVATE cxx_std_20)
target_compile_options(rp_tests PRIVATE -Wall -Wextra)
target_link_libraries(rp_tests PRIVATE GTest::gtest_main glm::glm)

include(GoogleTest)
gtest_discover_tests(rp_tests EXTRA_ARGS --gtest_color=yes)
