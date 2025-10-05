cmake_minimum_required(VERSION 3.28)
project(RPEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

# make CMAKE_BUILD_TYPE case-insensitive
string(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

# executable name
set(EXE_NAME ${CMAKE_PROJECT_NAME})

# set compile flags for build types
if(CMAKE_BUILD_TYPE STREQUAL "debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")
    set(EXE_NAME "RPEngineDebug")
    message(STATUS "Building in Debug mode")
elseif(CMAKE_BUILD_TYPE STREQUAL "release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
    set(EXE_NAME "RPEngine")
    message(STATUS "Building in Release mode")
endif()

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

add_executable(${EXE_NAME} ${SOURCES})
target_compile_features(${EXE_NAME} PRIVATE cxx_std_17)
target_include_directories(${EXE_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(${EXE_NAME} PRIVATE -Wall -Wextra)
target_link_libraries(${EXE_NAME} PRIVATE SFML::Graphics)

file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})

add_custom_target(debug
    COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=debug -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
    COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
    USES_TERMINAL
    COMMENT "Configuring to Debug mode and building ..."
)

add_custom_target(release
    COMMAND "${CMAKE_COMMAND}" -DCMAKE_BUILD_TYPE=release -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}"
    COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}"
    USES_TERMINAL
    COMMENT "Configuring to Release mode and building ..."
)
