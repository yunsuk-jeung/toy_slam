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
}

bool SlamApp::prepare() {
  if (!App::prepare()) {
    return false;
  }
  ImGui::Object::RenderImpl impl = []() {
    //ImGui::Begin("test2");
    ImGui::Text("ffffffffffffffffffffff");
  };
 ImGui::Object::Ptr obj = std::make_shared<ImGui::Object>(impl);
  mGUI->addImGuiObjects(obj);

  return true;
}

void SlamApp::run() {
  mSensor->start();

  mEndApplication = false;
  while (!mEndApplication) { onRender(); }

  mSensor->stop();
}

void SlamApp::createPipelineLayouts() {
  App::createPipelineLayouts();

  ShaderSourceType type      = ShaderSourceType::STRING;
  std::string      name      = "basic";
  const auto&      basicVert = shader::basicPointVert;
  const auto&      basicFrag = shader::basicFrag;

  auto* vert = ResourcePool::loadShader("basicPoint",
                                        mDevice.get(),
                                        type,
                                        vk::ShaderStageFlagBits::eVertex,
                                        basicVert);

  auto* frag = ResourcePool::loadShader(name,
                                        mDevice.get(),
                                        type,
                                        vk::ShaderStageFlagBits::eFragment,
                                        basicFrag);

  ResourcePool::addPipelineLayout(mDevice.get(), vert, frag);
}

void SlamApp::createPipelines() {
  App::createPipelines();
  using RP               = ResourcePool;
  auto* basicPointLayout = RP::requestPipelineLayout("basicPoint_vert_basic_frag");

  auto* basicPointPL = new PointBasicPipeline("point_basic",
                                              mDevice.get(),
                                              mRenderContext.get(),
                                              mVkRenderPass,
                                              basicPointLayout);
  basicPointPL->prepare();
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

  if (mGUI)
    mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

  cmd.endRenderPass();
  cmd.end();
}

void SlamApp::updateUniform(int idx) {
  updateCameraUniform(idx);
}

}  //namespace vkl
