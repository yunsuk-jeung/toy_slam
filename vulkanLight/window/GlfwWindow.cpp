#include <iostream>

#define GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "GlfwWindow.h"
#include "app/App.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"

#ifdef _DEBUG
#define IMGUI_VULKAN_DEBUG_REPORT
#endif

namespace vkl {

namespace {
void glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}
void window_size_callback(GLFWwindow* window, int width, int height) {
  if (auto app = reinterpret_cast<App*>(glfwGetWindowUserPointer(window))) {
    app->onWindowResized(width, height);
  }
}
}  //namespace

GlfwWindow::GlfwWindow(WindowInfo& _info, App* app) : Window(_info, app) {
  createWindow();
}

GlfwWindow::~GlfwWindow() {
  glfwDestroyWindow(window);
  glfwTerminate();
}

VkSurfaceKHR GlfwWindow::createSurface(Instance* instance) {

  return createSurface(instance->getVkInstance(), VK_NULL_HANDLE);
};

VkSurfaceKHR GlfwWindow::createSurface(VkInstance instance, VkPhysicalDevice device) {
  if (instance == VK_NULL_HANDLE || !window) {
    return VK_NULL_HANDLE;
  }

  VkSurfaceKHR surface;

  VkResult errCode = glfwCreateWindowSurface(instance, window, NULL, &surface);

  if (errCode != VK_SUCCESS) {
    return nullptr;
  }

  return surface;
}

std::vector<const char*> GlfwWindow::getRequiredSurfaceExtension() {
  uint32_t     glfw_extension_count{0};
  const char** names = glfwGetRequiredInstanceExtensions(&glfw_extension_count);
  return {names, names + glfw_extension_count};
}

void GlfwWindow::pollEvents() {
  glfwPollEvents();
}

void GlfwWindow::createWindow() {
  glfwSetErrorCallback(glfw_error_callback);

  if (!glfwInit()) return;

  //Create window with Vulkan context
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  switch (windowInfo.mode) {
  case WindowInfo::Mode::Fullscreen: {
    auto*       monitor      = glfwGetPrimaryMonitor();
    const auto* mode         = glfwGetVideoMode(monitor);
    windowInfo.extent.width  = mode->width;
    windowInfo.extent.height = mode->height;

    window = glfwCreateWindow(mode->width,
                              mode->height,
                              windowInfo.title.c_str(),
                              monitor,
                              NULL);
    break;
  }

  case WindowInfo::Mode::FullscreenBorderless: {
    auto*       monitor      = glfwGetPrimaryMonitor();
    const auto* mode         = glfwGetVideoMode(monitor);
    windowInfo.extent.width  = mode->width;
    windowInfo.extent.height = mode->height;

    glfwWindowHint(GLFW_RED_BITS, mode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);

    window = glfwCreateWindow(mode->width,
                              mode->height,
                              windowInfo.title.c_str(),
                              monitor,
                              NULL);

    break;
  }

  case WindowInfo::Mode::FullscreenStretch: {
    throw std::runtime_error("Cannot support stretch mode on this platform.");
    break;
  }

  default:
    window = glfwCreateWindow(windowInfo.extent.width,
                              windowInfo.extent.height,
                              windowInfo.title.c_str(),
                              NULL,
                              NULL);
    break;
  }

  if (!glfwVulkanSupported()) {
    printf("GLFW: Vulkan Not Supported\n");
    return;
  }

  if (!window) {
    std::cout << "Window create fail" << std::endl;
    return;
  }

  glfwSetWindowUserPointer(window, app);
  glfwSetWindowSizeCallback(window, window_size_callback);
}

void GlfwWindow::prepareGUI() {
  ImGui_ImplGlfw_InitForVulkan(window, true);
}

void GlfwWindow::newGUIFrame() {
  ImGui_ImplGlfw_NewFrame();
}

void GlfwWindow::endGUIFrame() {
  //this function is not used
  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
  }
}

void GlfwWindow::endGUI() {
  ImGui_ImplGlfw_Shutdown();
}

}  //namespace vkl