#ifndef RPENGINE_RENDER_WINDOW_HPP
#define RPENGINE_RENDER_WINDOW_HPP

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <string>

namespace render {

class Window {
 public:
  Window(int width, int height, const std::string& title);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;

  bool shouldClose() const noexcept;
  void pollEvents() const noexcept;
  void swapBuffers() const noexcept;

  // logical window size in units ImGui/GLFW input events use. NOT necessarily
  // physical pixels under fractional/HiDPI content scale
  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }

  int framebufferWidth() const noexcept { return fbWidth_; }
  int framebufferHeight() const noexcept { return fbHeight_; }

  GLFWwindow* handle() const noexcept { return window_; }

 private:
  static void framebufferSizeCallback(GLFWwindow* window, int w, int h);
  static void windowSizeCallback(GLFWwindow* window, int w, int h);

  GLFWwindow* window_ = nullptr;
  int width_;
  int height_;
  int fbWidth_;
  int fbHeight_;
};

}  // namespace render

#endif
