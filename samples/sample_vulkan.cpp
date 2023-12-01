#include <iostream>
#include "VkLogger.h"
#include "GlfwWindow.h"
#include "Device.h"
#include "GUI.h"
#include "VkShaderUtil.h"
#include "App.h"
#include "BasicRenderer.h"
#include "ResourcePool.h"
#include "shaders.h"
#include "PipelineLayout.h"
#include "ShaderModule.h"
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

    std::string basicPipeline = "default_basic_vert_basic_frag";
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
    createDefaultBasicPipeline();
  }

  void createDefaultBasicPipeline() {
    auto* basicPipelineLayout = ResourcePool::requestPipelineLayout(
      "basic_vert_basic_frag");
    auto& shaders = basicPipelineLayout->getShaderModules();

    std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
      {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
      {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
    };

    std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
      {0, sizeof(VertexColorNuv), vk::VertexInputRate::eVertex}
    };

    std::vector<vk::VertexInputAttributeDescription> attributeDescription{
      {0, 0, vk::Format::eR32G32B32Sfloat,                 0},
      {1, 0, vk::Format::eR32G32B32Sfloat, sizeof(float) * 3},
      {2, 0,    vk::Format::eR32G32Sfloat, sizeof(float) * 6}
    };

    vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                        vertBindingDescription,
                                                        attributeDescription);
    vk::PipelineInputAssemblyStateCreateInfo
      inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);

    auto&                               scProps = mRenderContext->getContextProps();
    vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);

    vk::PipelineRasterizationStateCreateInfo rasterizationState;
    rasterizationState.polygonMode = vk::PolygonMode::eFill;
    rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
    rasterizationState.frontFace   = vk::FrontFace::eClockwise;
    rasterizationState.lineWidth   = 1.0f;

    vk::PipelineMultisampleStateCreateInfo multisample_state({},
                                                             vk::SampleCountFlagBits::e1);

    vk::PipelineDepthStencilStateCreateInfo depthStencilState;
    depthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
    depthStencilState.depthTestEnable  = true;
    depthStencilState.depthWriteEnable = true;
    depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
    depthStencilState.front            = depthStencilState.back;

    vk::PipelineColorBlendAttachmentState blendAttachmentState;
    blendAttachmentState.colorWriteMask = vk::ColorComponentFlagBits::eR
                                          | vk::ColorComponentFlagBits::eG
                                          | vk::ColorComponentFlagBits::eB
                                          | vk::ColorComponentFlagBits::eA;

    vk::PipelineColorBlendStateCreateInfo colorBlendState({},
                                                          false,
                                                          {},
                                                          blendAttachmentState);

    std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
                                                           vk::DynamicState::eScissor};

    vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);

    vk::GraphicsPipelineCreateInfo pipelineCI({},
                                              shaderStageCIs,
                                              &inputStateCI,
                                              &inputAssemCI,
                                              {},
                                              &viewport_state,
                                              &rasterizationState,
                                              &multisample_state,
                                              &depthStencilState,
                                              &colorBlendState,
                                              &dynamicState,
                                              basicPipelineLayout->vk(),
                                              mVkRenderPass,
                                              {},
                                              {},
                                              -1);

    vk::Pipeline vkPipeline;
    vk::Result   result;

    std::tie(result, vkPipeline) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                        pipelineCI);
    VK_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");

    std::string name = "default_" + basicPipelineLayout->getName();

    VklLogD("created pipeline: {}", name);
    ResourcePool::addPipeline(name, vkPipeline);
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
  app.prepare();

  app.run();
  return 0;
}