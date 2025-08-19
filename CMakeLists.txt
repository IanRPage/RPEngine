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

include_directories(include)

set(SOURCES
    src/main.cpp
    src/Particle.cpp
    src/Simulator.cpp
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE inlcude)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -O2 -O0 -g)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SFML::Graphics)
