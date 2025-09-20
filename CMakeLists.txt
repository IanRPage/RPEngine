cmake_minimum_required(VERSION 3.28)
project(RPEngine2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.1
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

set(SOURCES
    src/main.cpp
    src/Simulator.cpp
    src/ui/Renderer.cpp
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -O2 -O0 -g -DENABLE_PROFILING=0)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SFML::Graphics)

file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})

# QuadTree benchmark executable
add_executable(quadtree_benchmark src/quadtree_benchmark.cpp)
target_compile_features(quadtree_benchmark PRIVATE cxx_std_17)
target_include_directories(quadtree_benchmark PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(quadtree_benchmark PRIVATE -Wall -Wextra -O2 -g)

# Profiled simulation executable
add_executable(rpengine_profiled src/main_profiled.cpp src/Simulator.cpp src/ui/Renderer.cpp)
target_compile_features(rpengine_profiled PRIVATE cxx_std_17)
target_include_directories(rpengine_profiled PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(rpengine_profiled PRIVATE -Wall -Wextra -O2 -g)
target_link_libraries(rpengine_profiled PRIVATE SFML::Graphics)
