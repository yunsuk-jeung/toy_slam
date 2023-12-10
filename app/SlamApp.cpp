#include "Sensor.h"
#include "Window.h"
#include "GUI.h"
#include "Device.h"
#include "VkShaderUtil.h"
#include "shaders.h"
#include "ResourcePool.h"
#include "PointBasicPipeline.h"
#include "PointCloudRenderer.h"
#include "ImGuiObject.h"
#include "SlamApp.h"

namespace vkl {
SlamApp::SlamApp()
  : mSensor{nullptr} {
  mName       = "SLAM Application";
  mApiVersion = VK_API_VERSION_1_1;
  GLSLCompiler::set_target_environment(glslang::EShTargetSpv, glslang::EShTargetSpv_1_3);
}

SlamApp ::~SlamApp() {
  vk::Device vkDevice = mDevice->vk();
  vkDevice.waitIdle();

  mPointCloudRenderer.reset();
}

bool SlamApp::prepare() {
  if (!App::prepare()) {
    return false;
  }

  ImGui::Object::RenderImpl impl = []() {
    ImGui::Begin("test2");
    ImGui::Text("ffffffffffffffffffffff");
    ImGui::End();
  };
  ImGui::Object::Ptr obj = std::make_shared<ImGui::Object>(impl);
  mGUI->addImGuiObjects(obj);

  mPointCloudRenderer = std::make_unique<PointCloudRenderer>();
  mPointCloudRenderer->setCamUB(mCameraUB.get());
  mPointCloudRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               "point_basic");

  return true;
}

void SlamApp::run() {
  mSensor->start();

  mEndApplication = false;
  while (!mEndApplication) { onRender(); }

  mSensor->stop();
}

void SlamApp::createPipelines() {
  App::createPipelines();

  ShaderSourceType type           = ShaderSourceType::STRING;
  std::string      name           = "basic";
  const auto&      basicPointVert = shader::basicPointVert;
  const auto&      basicFrag      = shader::basicFrag;

  using RP   = ResourcePool;
  auto* vert = RP::loadShader("point_basic",
                              mDevice.get(),
                              type,
                              vk::ShaderStageFlagBits::eVertex,
                              basicPointVert);

  auto* frag = RP::loadShader(name,
                              mDevice.get(),
                              type,
                              vk::ShaderStageFlagBits::eFragment,
                              basicFrag);

  auto* basicPointLayout = RP::addPipelineLayout(mDevice.get(), vert, frag);

  RP::addPipeline<PointBasicPipeline>("point_basic",
                                      mDevice.get(),
                                      mRenderContext.get(),
                                      mVkRenderPass,
                                      basicPointLayout);
}

void SlamApp::onRender() {
  mWindow->pollEvents();

  if (mGUI) {
    mGUI->onRender();
  }

  updateUniform(mCurrBufferingIdx);

  prepareFrame();
  buildCommandBuffer();
  presentFrame();
}

void SlamApp::buildCommandBuffer() {
  auto cmd = beginCommandBuffer();
  beginRenderPass(cmd);

  mPointCloudRenderer->buildCommandBuffer(cmd, mCurrBufferingIdx);

  if (mGUI)
    mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

  cmd.endRenderPass();
  cmd.end();
}

void SlamApp::updateUniform(int idx) {
  updateCameraUniform(idx);
}

}  //namespace vkl
