cmake_minimum_required(VERSION 3.28)
project(RPEngine LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the build type (Debug or Release)" FORCE)
endif()

option(BUILD_TESTS "Build unit tests" ON)

add_subdirectory(rpe-physics)
add_subdirectory(rpe-render)
add_subdirectory(rpengine)

add_custom_target(debug
  COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=Debug
          -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
  COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
  USES_TERMINAL
  COMMENT "Configuring to Debug mode and building ..."
)

add_custom_target(release
  COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=Release
          -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
  COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
  USES_TERMINAL
  COMMENT "Configuring to Release mode and building ..."
)

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
