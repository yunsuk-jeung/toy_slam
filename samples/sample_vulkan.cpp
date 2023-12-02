#include <iostream>
#include "VklLogger.h"
#include "GlfwWindow.h"
#include "Device.h"
#include "GUI.h"
#include "VkShaderUtil.h"
#include "App.h"
#include "BasicRenderer.h"
#include "ResourcePool.h"
#include "shaders.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "RenderContext.h"
#include "VkError.h"
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
    if (!App::prepare()) { return false; }

    std::string basicPipeline = "triangle_basic";
    mBasicRenderer.prepare(mDevice.get(),
                           mRenderContext.get(),
                           mVkDescPool,
                           mVkRenderPass,
                           basicPipeline);

    return true;
  }

  void run() override {
    mEndApplication = false;
    while (!mEndApplication) { onRender(); }
  }

protected:
  void createPipelineLayouts() override {
    App::createPipelineLayouts();

    ShaderSourceType   type           = ShaderSourceType::STRING;
    std::string        name           = "basic";
    const std::string& vertexShader   = shader::basicVert;
    const std::string& fragmentShader = shader::basicFrag;

    auto* vert = ResourcePool::loadShader(name,
                                          mDevice.get(),
                                          type,
                                          vk::ShaderStageFlagBits::eVertex,
                                          vertexShader);

    auto* frag = ResourcePool::loadShader(name,
                                          mDevice.get(),
                                          type,
                                          vk::ShaderStageFlagBits::eFragment,
                                          fragmentShader);

    ResourcePool::addPipelineLayout(mDevice.get(), vert, frag);
  }

  void createPipelines() override {
    App::createPipelines();

    auto* basicPipelineLyout = ResourcePool::requestPipelineLayout(
      "basic_vert_basic_frag");

    auto* basicTrianglePL = new BasicTraianglePipeline("triangle_basic",
                                                       mDevice.get(),
                                                       mRenderContext.get(),
                                                       mVkRenderPass,
                                                       basicPipelineLyout);
    basicTrianglePL->prepare();
  }

  void onRender() override {
    mWindow->pollEvents();

    if (mGUI) { mGUI->onRender(); }

    updateUniform(mCurrBufferingIdx);

    prepareFrame();
    buildCommandBuffer();
    presentFrame();
  }

  void buildCommandBuffer() override {
    auto cmd = beginCommandBuffer();
    beginRenderPass(cmd);

    if (mGUI) mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

    cmd.endRenderPass();
    cmd.end();
  }

  void updateUniform(int idx) override { updateCameraUniform(idx); }

protected:
  BasicRenderer mBasicRenderer;
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

  app.setShaderPath("D:/workspaceD/toy_vio/libvulkanlight/shaders");
  app.setResourcePath("D:/workspaceD/toy_vio/libvulkanlight/shaders");

  app.prepare();

  app.run();
  return 0;
}