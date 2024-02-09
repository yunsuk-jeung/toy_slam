#include <iostream>
#include "VklLogger.h"
#include "GlfwWindow.h"
#include "Device.h"
#include "GUI.h"
#include "VklShaderUtil.h"
#include "App.h"
#include "ResourcePool.h"
#include "ShaderModule.h"
#include "ShaderPool.h"
#include "PipelineLayout.h"
#include "GraphicsPipeline.h"
#include "RenderContext.h"
#include "VklError.h"
#include "Utils.h"

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
    vk::Device vkDevice = mDevice->vk();
    vkDevice.waitIdle();
  }
  void onWindowResized(int w, int h, int orientation = 0) override {
    App::onWindowResized(w, h, orientation);
  }

  bool prepare() override {
    if (!App::prepare()) {
      return false;
    }

    return true;
  }

  void run() override {
    mEndApplication = false;
    while (!mEndApplication) {
      onRender();
    }
  }

protected:
  void createPipelines() override {
    App::createPipelines();

    ShaderSourceType   type         = ShaderSourceType::STRING;
    std::string        name         = "pcucmst";
    const std::string& vertexShader = ShaderPool::requestShaderStr(
      "pos_col_nuv_mvp_m_vert");
    const std::string& fragmentShader = ShaderPool::requestShaderStr(
      "col_uv_mat_tex_rgba_frag");
  }

  void onRender() override {
    mWindow->pollEvents();

    if (mGUI) {
      mGUI->onRender();
    }

    updateUniform(mCurrBufferingIdx);

    prepareFrame();
    buildCommandBuffer();
    presentFrame();
  }

  void buildCommandBuffer() override {
    auto cmd = beginCommandBuffer();
    beginRenderPass(cmd);

    if (mGUI)
      mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

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
  app.addShaderPath("F:/transfer/toy_slam/libvulkanlight/shaders");
  app.addAssetPath("F:/transfer/toy_slam/libvulkanlight/shaders");

  app.addShaderPath("D:/workspaceD/toy_vio/libvulkanlight/shaders");
  app.addAssetPath("D:/workspaceD/toy_vio/libvulkanlight/shaders");

  app.prepare();

  app.run();
  return 0;
}