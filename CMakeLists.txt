cmake_minimum_required(VERSION 3.28)
project(RPEngine LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# put all built executables in <build-dir>/bin, regardless of source
# subdirectory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
        "Choose the type of build (Debug or Release)" FORCE)
endif()

# executable name
set(EXE_NAME RPEngine)

# unit tests
option(BUILD_TESTS "Build unit tests" ON)

# -------- fetch content stuff --------
include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v2.0.8
    GIT_SHALLOW ON
    SOURCE_SUBDIR cmake
)
FetchContent_MakeAvailable(glad)

glad_add_library(glad_gl_core_41 REPRODUCIBLE API gl:core=4.1)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.3
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw glad_gl_core_41)
# -------------------------------------

set(SOURCES
    src/main.cpp
    src/math/Transform.cpp
    src/math/Rotation.cpp
    src/math/Constraints2D.cpp
    src/collision/Shapes.cpp
    src/collision/MassProperties.cpp
    src/core/BodyStore.cpp
    src/broadphase/DynamicBVH.cpp
    src/broadphase/PairCache.cpp
    src/broadphase/NaiveBroadphase.cpp
    src/broadphase/GridBroadphase.cpp
    src/broadphase/DynamicBVHBroadphase.cpp
)

add_executable(${EXE_NAME} ${SOURCES})
target_compile_features(${EXE_NAME} PRIVATE cxx_std_20)
target_include_directories(${EXE_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(${EXE_NAME} PRIVATE
    -Wall -Wextra
    $<$<CONFIG:Debug>:-O0 -g -DDEBUG>
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
)
target_link_libraries(${EXE_NAME} PRIVATE glfw glad_gl_core_41 glm::glm imgui)

# file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})

add_custom_target(debug
    COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=Debug -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
    COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
    USES_TERMINAL
    COMMENT "Configuring to Debug mode and building ..."
)

add_custom_target(release
    COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=Release -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
    COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
    USES_TERMINAL
    COMMENT "Configuring to Release mode and building ..."
)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
