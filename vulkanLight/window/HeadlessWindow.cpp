#include "HeadlessWindow.h"

namespace vkl {
HeadlessWindow::HeadlessWindow(WindowInfo& _info, Application* app)
  : Window(_info, app) {}

HeadlessWindow::~HeadlessWindow() {}

VkSurfaceKHR HeadlessWindow::createSurface(Instance* instance) {
  return VK_NULL_HANDLE;
}
VkSurfaceKHR HeadlessWindow::createSurface(VkInstance       instance,
                                           VkPhysicalDevice physical_device) {
  return VK_NULL_HANDLE;
}
std::vector<const char*> HeadlessWindow::getRequiredSurfaceExtension() {
  return {};
}

void HeadlessWindow::pollEvents() {}

void HeadlessWindow::prepareGUI() {}

void HeadlessWindow::newGUIFrame() {}

void HeadlessWindow::endGUIFrame() {}

void HeadlessWindow::endGUI() {}

void HeadlessWindow::createWindow() {}

}  //namespace vkl