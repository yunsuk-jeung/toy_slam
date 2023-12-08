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
    if (!App::prepare()) {
      return false;
    }

    //std::string pcucmstPipeline = "triangle_pcucmst";
    //mBasicRenderer.prepare(mDevice.get(),
    //                       mRenderContext.get(),
    //                       mVkDescPool,
    //                       mVkRenderPass,
    //                       pcucmstPipeline);

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
    std::string        name           = "pcucmst";
    const std::string& vertexShader   = shader::pose_color_uv_camera_model_vert;
    const std::string& fragmentShader = shader::color_uv_material_sampler_texture;

    //auto* vert = ResourcePool::loadShader(name,
    //                                      mDevice.get(),
    //                                      type,
    //                                      vk::ShaderStageFlagBits::eVertex,
    //                                      vertexShader);

    //auto* frag = ResourcePool::loadShader(name,
    //                                      mDevice.get(),
    //                                      type,
    //                                      vk::ShaderStageFlagBits::eFragment,
    //                                      fragmentShader);

    //ResourcePool::addPipelineLayout(mDevice.get(), vert, frag);
  }

  void createPipelines() override {
    App::createPipelines();

    //auto* basicPipelineLyout = ResourcePool::requestPipelineLayout(
    //  "pcucmst_vert_pcucmst_frag");

    //auto* basicTrianglePL = new TextureMaterialPipeline("triangle_pcucmst",
    //                                                   mDevice.get(),
    //                                                   mRenderContext.get(),
    //                                                   mVkRenderPass,
    //                                                   basicPipelineLyout);
    //basicTrianglePL->prepare();
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
  app.addShaderPath("F:/transfer/toy_slam/libvulkanlight/shaders");
  app.addResourcePath("F:/transfer/toy_slam/libvulkanlight/shaders");

  app.addShaderPath("D:/workspaceD/toy_vio/libvulkanlight/shaders");
  app.addResourcePath("D:/workspaceD/toy_vio/libvulkanlight/shaders");

  app.prepare();

  app.run();
  return 0;
}