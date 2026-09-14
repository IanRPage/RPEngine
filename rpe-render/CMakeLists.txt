cmake_minimum_required(VERSION 3.28)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(rpe-render LANGUAGES CXX C)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
rpe_ensure_glm()
rpe_ensure_glfw()
rpe_ensure_glad()

add_library(rpe-render STATIC
  src/render/Camera.cpp
  src/render/InstancedBatch.cpp
  src/render/Mesh.cpp
  src/render/MeshLibrary.cpp
  src/render/ShaderProgram.cpp
  src/render/Window.cpp
)
add_library(RPEngine::Render ALIAS rpe-render)

target_compile_features(rpe-render PUBLIC cxx_std_20)
target_include_directories(rpe-render PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_options(rpe-render PRIVATE
  -Wall
  -Wextra
  $<$<CONFIG:Debug>:-O0;-g>
  $<$<CONFIG:Release>:-O3>
)
target_link_libraries(rpe-render
  PUBLIC glm::glm glfw glad_gl_core_41
)
