include_guard(GLOBAL)

include(FetchContent)

# Let CI/container builds use their pre-populated FetchContent cache without
# baking an environment-specific path into the project.
if(DEFINED ENV{FETCHCONTENT_BASE_DIR})
  set(FETCHCONTENT_BASE_DIR "$ENV{FETCHCONTENT_BASE_DIR}" CACHE PATH "" FORCE)
endif()
if(DEFINED ENV{FETCHCONTENT_FULLY_DISCONNECTED})
  set(FETCHCONTENT_FULLY_DISCONNECTED
      "$ENV{FETCHCONTENT_FULLY_DISCONNECTED}" CACHE BOOL "" FORCE)
endif()

function(rpe_ensure_glm)
  if(TARGET glm::glm)
    return()
  endif()

  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.3
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM
  )
  FetchContent_MakeAvailable(glm)
endfunction()

function(rpe_ensure_glfw)
  if(TARGET glfw)
    return()
  endif()

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
endfunction()

function(rpe_ensure_glad)
  if(TARGET glad_gl_core_41)
    return()
  endif()

  FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v2.0.8
    GIT_SHALLOW ON
    SOURCE_SUBDIR cmake
  )
  FetchContent_MakeAvailable(glad)
  glad_add_library(glad_gl_core_41 REPRODUCIBLE API gl:core=4.1)
endfunction()

function(rpe_ensure_imgui)
  if(TARGET imgui)
    return()
  endif()

  rpe_ensure_glfw()
  rpe_ensure_glad()
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/imgui_demo.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
  )
  target_include_directories(imgui PUBLIC
    "${imgui_SOURCE_DIR}"
    "${imgui_SOURCE_DIR}/backends"
  )
  target_link_libraries(imgui PUBLIC glfw glad_gl_core_41)
endfunction()

function(rpe_ensure_stb)
  if(TARGET rpe-stb)
    return()
  endif()

  FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(stb)
  add_library(rpe-stb INTERFACE)
  target_include_directories(rpe-stb SYSTEM INTERFACE "${stb_SOURCE_DIR}")
endfunction()

function(rpe_ensure_googletest)
  if(TARGET GTest::gtest_main)
    return()
  endif()

  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(googletest)
endfunction()
