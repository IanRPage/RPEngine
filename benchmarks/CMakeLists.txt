cmake_minimum_required(VERSION 3.28)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(rpe-benchmarks LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

  include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../rpe-physics"
                   "${CMAKE_CURRENT_BINARY_DIR}/rpe-physics")
else()
  include("${CMAKE_CURRENT_LIST_DIR}/../cmake/Dependencies.cmake")
endif()

rpe_ensure_googlebenchmark()

add_executable(rp_bench bench_physics.cpp)
target_compile_features(rp_bench PRIVATE cxx_std_20)
target_compile_options(rp_bench PRIVATE -Wall -Wextra)
target_link_libraries(rp_bench PRIVATE rpe-physics benchmark::benchmark)
