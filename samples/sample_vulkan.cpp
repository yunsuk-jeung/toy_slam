#include <iostream>
#include "Logger.h"
#include "GlfwWindow.h"
#include "Device.h"
#include "GUI.h"
#include "VkShaderUtil.h"
#include "App.h"
namespace vkl {
class SampleApp : public App {
public:
  SampleApp() {
    mName       = "Compute Application";
    mApiVersion = VK_API_VERSION_1_1;
    GLSLCompiler::set_target_environment(glslang::EShTargetSpv,
                                         glslang::EShTargetSpv_1_3);
  }
  ~SampleApp() {
    vk::Device vkDevice = mDevice->getVkDevice();
    vkDevice.waitIdle();
  }
  void onWindowResized(int w, int h, int orientation = 0) override {
    App::onWindowResized(w, h, orientation);
  }

  bool prepare() override {
    if (!App::prepare()) { return false; }

    return true;
  }

  void run() override {
    mEndApplication = false;
    while (!mEndApplication) { onRender(); }
  }

  void onRender() override {
    mWindow->pollEvents();

    if (mGui) { mGui->onRender(); }

    updateUniform(mCurrBufferingIdx);

    prepareFrame();
    buildCommandBuffer();
    presentFrame();
  }

  void buildCommandBuffer() override {
    auto cmd = beginCommandBuffer();
    beginRenderPass(cmd);

    if (mGui) mGui->buildCommandBuffer(cmd, mCurrBufferingIdx);

    cmd.endRenderPass();
    cmd.end();
  }
  void updateUniform(int idx) override { updateCameraUniform(idx); }

protected:
};
}  //namespace vkl

int main() {
  vkl::WindowInfo winInfo{
    "Sample window",
    vkl::WindowInfo::Mode::Default,
    true,
    vkl::WindowInfo::Vsync::ON,
    vkl::WindowInfo::Orientation::Landscape,
    {1280, 720}
  };

  vkl::SampleApp app;

  std::unique_ptr<vkl::Window> glfwWindow(new vkl::GlfwWindow(winInfo, &app));

  app.registerWindow(glfwWindow);
  app.setShaderPath("F:/transfer/toy_slam/libvulkanlight/shaders");
  app.setResourcePath("F:/transfer/toy_slam/libvulkanlight/shaders");
  app.prepare();

  app.run();
  return 0;
}