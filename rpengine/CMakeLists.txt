cmake_minimum_required(VERSION 3.28)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(rpengine LANGUAGES CXX C)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")

if(NOT TARGET rpe-physics)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpe-physics"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpe-physics")
endif()
if(NOT TARGET rpe-render)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpe-render"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpe-render")
endif()
rpe_ensure_stb()
rpe_ensure_imgui()

get_filename_component(RPE_ASSETS_DIR
  "${CMAKE_CURRENT_LIST_DIR}/../assets" ABSOLUTE
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/gui/AssetPaths.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/include/gui/AssetPaths.hpp"
  @ONLY
)

add_library(rpengine STATIC
  src/sim/Simulator.cpp
  src/gui/RenderableStore.cpp
  src/gui/HullMeshCache.cpp
  src/gui/ImguiController.cpp
  src/gui/Renderer.cpp
  src/gui/StbImageWriteImpl.cpp
)
add_library(RPEngine::Engine ALIAS rpengine)

target_compile_features(rpengine PUBLIC cxx_std_20)
target_include_directories(rpengine
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated/include"
)
target_compile_options(rpengine PRIVATE
  -Wall
  -Wextra
  $<$<CONFIG:Debug>:-O0;-g>
  $<$<CONFIG:Release>:-O3>
)
target_link_libraries(rpengine
  PUBLIC rpe-physics rpe-render
  PRIVATE imgui rpe-stb
)

add_executable(RPEngine src/main.cpp)
target_compile_features(RPEngine PRIVATE cxx_std_20)
target_compile_options(RPEngine PRIVATE -Wall -Wextra)
target_link_libraries(RPEngine PRIVATE rpengine)
