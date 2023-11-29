#pragma once

#include "window/Window.h"

struct GLFWwindow;
namespace vkl {
class GlfwWindow : public Window {
public:
  GlfwWindow() = delete;
  GlfwWindow(WindowInfo& _info, App* app);
  ~GlfwWindow();

  VkSurfaceKHR createSurface(Instance* instance) override;
  VkSurfaceKHR createSurface(VkInstance       instance,
                             VkPhysicalDevice physical_device) override;

  std::vector<const char*> getRequiredSurfaceExtension() override;

  void pollEvents() override;
  void prepareGUI() override;
  void newGUIFrame() override;
  void endGUIFrame() override;
  void endGUI() override;

protected:
  void createWindow() override;

protected:
  GLFWwindow* window{nullptr};
};
}  //namespace vkl