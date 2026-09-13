#include <render/Window.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdio>
#include <stdexcept>

namespace render {

namespace {
void glfwErrorCallback(int error, const char* description) {
  std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}
}  // namespace

Window::Window(int width, int height, const std::string& title)
    : width_(width), height_(height), fbWidth_(width), fbHeight_(height) {
  glfwSetErrorCallback(glfwErrorCallback);

  if (!glfwInit()) { throw std::runtime_error("Window: glfwInit() failed"); }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  window_ = glfwCreateWindow(width_, height_, title.c_str(), nullptr, nullptr);
  if (window_ == nullptr) {
    glfwTerminate();
    throw std::runtime_error("Window: glfwCreateWindow() failed");
  }

  glfwMakeContextCurrent(window_);
  glfwSetWindowUserPointer(window_, this);
  glfwSetFramebufferSizeCallback(window_, framebufferSizeCallback);
  glfwSetWindowSizeCallback(window_, windowSizeCallback);

  if (!gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress))) {
    glfwDestroyWindow(window_);
    glfwTerminate();
    throw std::runtime_error(
        "Window: gladLoadGL() failed to load OpenGL 4.1 core function "
        "pointers");
  }

  glfwGetFramebufferSize(window_, &fbWidth_, &fbHeight_);
  glfwGetWindowSize(window_, &width_, &height_);

  glViewport(0, 0, fbWidth_, fbHeight_);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;

  bool glfwBackendReady = false;
  bool openglBackendReady = false;
  try {
    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
      throw std::runtime_error(
          "Window: ImGui_ImplGlfw_InitForOpenGL() failed");
    }
    glfwBackendReady = true;

    if (!ImGui_ImplOpenGL3_Init("#version 410 core")) {
      throw std::runtime_error("Window: ImGui_ImplOpenGL3_Init() failed");
    }
    openglBackendReady = true;
  } catch (...) {
    if (openglBackendReady) { ImGui_ImplOpenGL3_Shutdown(); }
    if (glfwBackendReady) { ImGui_ImplGlfw_Shutdown(); }
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
    throw;
  }
}

Window::~Window() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  if (window_ != nullptr) { glfwDestroyWindow(window_); }
  glfwTerminate();
}

bool Window::shouldClose() const noexcept {
  return glfwWindowShouldClose(window_) != 0;
}

void Window::pollEvents() const noexcept { glfwPollEvents(); }

void Window::swapBuffers() const noexcept { glfwSwapBuffers(window_); }

void Window::framebufferSizeCallback(GLFWwindow* window, int w, int h) {
  auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
  self->fbWidth_ = w;
  self->fbHeight_ = h;
  glViewport(0, 0, w, h);
}

void Window::windowSizeCallback(GLFWwindow* window, int w, int h) {
  auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
  self->width_ = w;
  self->height_ = h;
}

}  // namespace render
