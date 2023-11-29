#pragma once

#include "window/Window.h"

namespace vkl {
class HeadlessWindow : public Window {
public:
  HeadlessWindow() = delete;
  HeadlessWindow(WindowInfo& _info, Application* app);
  ~HeadlessWindow();

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
};
}  //namespace vkl