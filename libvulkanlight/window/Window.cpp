#include "Window.h"

namespace vkl {
Window::Window(WindowInfo& info, App* _app)
  : windowInfo(info)
  , app(_app) {
  //createWindow();
}
void Window::updateExtent(uint32_t w, uint32_t h) {
  windowInfo.extent.width  = w;
  windowInfo.extent.height = h;
}

void Window::pollEvents() {}

}  //namespace vkl