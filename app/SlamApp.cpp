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

  std::string filePath = __FILE__;
  size_t      pos      = filePath.find_last_of("\\/");
  std::string dir      = (std::string::npos == pos) ? "" : filePath.substr(0, pos);
  VklLogD("app directory : {}", dir);

  addShaderPath(dir);
  addAssetPath(dir);
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

  return true;
}

void SlamApp::run() {
  mSensor->start();

  mEndApplication = false;
  while (!mEndApplication) { onRender(); }

  mSensor->stop();
}

void SlamApp::createPipelines() {
  ShaderSourceType type = ShaderSourceType::STRING_FILE;

  constexpr char vert[]     = "basicPoint";
  constexpr char vertFile[] = "basicPoint.vert";

  constexpr char frag[]     = "basic";
  constexpr char fragFile[] = "basic.frag";

  using RP         = ResourcePool;
  auto* vertShader = RP::loadShader(vert,
                                    mDevice.get(),
                                    type,
                                    vk::ShaderStageFlagBits::eVertex,
                                    vertFile);

  auto* fragShader = RP::loadShader(frag,
                                    mDevice.get(),
                                    type,
                                    vk::ShaderStageFlagBits::eFragment,
                                    fragFile);

  auto* basicPointLayout = RP::addPipelineLayout(mDevice.get(), vertShader, fragShader);

  constexpr char pointPipelinename[] = "point_basic";

  auto* pointPipeline = RP::addPipeline<PointBasicPipeline>(pointPipelinename,
                                                            mDevice.get(),
                                                            mRenderContext.get(),
                                                            mVkRenderPass,
                                                            basicPointLayout);

  mPointCloudRenderer = std::make_unique<PointCloudRenderer>();
  mPointCloudRenderer->setCamUB(mCameraUB.get());
  mPointCloudRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               pointPipeline);
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
