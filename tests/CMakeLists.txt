include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(googletest)

file(GLOB_RECURSE ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "main\\.cpp$")
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "Renderer\\.cpp$")
list(FILTER ENGINE_SOURCES EXCLUDE REGEX "ImguiController\\.cpp$")

add_executable(rp_tests
    test_vec2.cpp
    test_aabb.cpp
    test_particle.cpp
    test_quadtree.cpp
    test_spatial_grid.cpp
    test_simulator.cpp
    test_iterative_solver.cpp
    test_constraints.cpp
    test_collision_math.cpp
    ${ENGINE_SOURCES}
)

target_include_directories(rp_tests PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(rp_tests PRIVATE -Wall -Wextra)
target_link_libraries(rp_tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(rp_tests EXTRA_ARGS --gtest_color=yes)
