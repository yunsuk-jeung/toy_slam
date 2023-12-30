#include <iostream>
#include <string>
#include <iomanip>
#include "Simulator.h"
#include "Window.h"
#include "GraphicsCamera.h"
#include "GUI.h"
#include "Device.h"
#include "VkShaderUtil.h"
#include "shaders.h"
#include "ResourcePool.h"
#include "SlamObjectPipeline.h"
#include "AxisRenderer.h"
#include "PointCloudRenderer.h"
#include "ImGuiObject.h"
#include "SLAMInfo.h"
#include "SLAMApp.h"

namespace vkl {
namespace {
template <typename Type, int Row>
static std::string eigenVec(const Eigen::Matrix<Type, Row, 1>& vec, int precision = 4) {
  Eigen::IOFormat CleanVec(precision, 0, ", ", "\n", "[", "]");

  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);  //Set the precision
  ss << vec.transpose().format(CleanVec);
  return ss.str();
}

template <typename Type, int Row, int Col>
static std::string eigenMat(const Eigen::Matrix<Type, Row, Col>& mat, int precision = 4) {
  Eigen::IOFormat CleanMat(precision, 0, ", ", "\n", "[", "]");

  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);  //Set the precision
  ss << "\n " << mat.transpose().format(CleanMat);
  return ss.str();
}
}  //namespace

SLAMApp::SLAMApp()
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

SLAMApp ::~SLAMApp() {
  vk::Device vkDevice = mDevice->vk();
  vkDevice.waitIdle();

  mPointCloudRenderer.reset();
}

bool SLAMApp::prepare() {
  if (!App::prepare()) {
    return false;
  }

  Eigen::Quaternionf Qxz = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(),
                                                              Eigen::Vector3f::UnitZ());
  mGraphicsCamera->cam.M.block<3, 3>(0, 0) = Qxz.toRotationMatrix();

  ((io::Simulator*)mSensor)->setContinuosMode(false);
  ImGui::Object::RenderImpl impl = [this]() {
    ImGui::Begin("Simulator");
    if (ImGui::Button("next image")) {
      ((io::Simulator*)mSensor)->sendImage();
    }
    ImGui::End();
  };

  ImGui::Object::Ptr obj = std::make_shared<ImGui::Object>(impl);
  mGUI->addImGuiObjects(obj);

  return true;
}

void SLAMApp::run() {
  mSensor->start();

  mEndApplication = false;
  while (!mEndApplication) {
    onRender();
  }

  mSensor->stop();
}

void SLAMApp::createRenderers() {
  createPointRenderer();
  createOriginRenderer();
  createAxisRenderer();
}

void SLAMApp::onRender() {
  mWindow->pollEvents();

  if (mGUI) {
    mGUI->onRender();
  }
  updateUniform(mCurrBufferingIdx);
  updateSLAMData();

  prepareFrame();
  buildCommandBuffer();
  presentFrame();
}

void SLAMApp::buildCommandBuffer() {
  auto cmd = beginCommandBuffer();
  beginRenderPass(cmd);

  mOriginAxisRenderer->buildCommandBuffer(cmd, mCurrBufferingIdx);
  mPointCloudRenderer->buildCommandBuffer(cmd, mCurrBufferingIdx);

  mAxisRenderer->buildCommandBuffer(cmd, mCurrBufferingIdx);

  if (mGUI)
    mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

  cmd.endRenderPass();
  cmd.end();
}

void SLAMApp::updateUniform(int idx) {
  updateCameraUniform(idx);
}

void SLAMApp::createPointRenderer() {
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

  auto* pointPipeline = RP::addPipeline<BasicPointPipeline>(pointPipelinename,
                                                            mDevice.get(),
                                                            mRenderContext.get(),
                                                            mVkRenderPass,
                                                            basicPointLayout);

  mPointCloudRenderer = std::make_unique<PointCloudRenderer>();
  mPointCloudRenderer->setCamUB(mCameraUB.get());
  mPointCloudRenderer->setPointCloud(&mLocalPointClouds);
  mPointCloudRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               pointPipeline);
}

void SLAMApp::createOriginRenderer() {
  ShaderSourceType type = ShaderSourceType::STRING_FILE;

  constexpr char vert[]     = "basicDynamic";
  constexpr char vertFile[] = "basic.vert";

  constexpr char frag[]     = "basic";
  constexpr char fragFile[] = "basic.frag";

  using RP              = ResourcePool;
  auto* vertShader      = RP::loadShader(vert,
                                    mDevice.get(),
                                    type,
                                    vk::ShaderStageFlagBits::eVertex,
                                    vertFile);
  auto& shaderResources = vertShader->getShaderResources();

  for (auto& resource : shaderResources) {
    if (resource.set != 1) {
      continue;
    }
    resource.mode = ShaderResourceMode::Dynamic;
  }

  auto* fragShader = RP::loadShader(frag,
                                    mDevice.get(),
                                    type,
                                    vk::ShaderStageFlagBits::eFragment,
                                    fragFile);

  auto* basicPointLayout = RP::addPipelineLayout(mDevice.get(), vertShader, fragShader);

  constexpr char linePipelinename[] = "basic_line";

  auto*           axisPipeline = RP::addPipeline<BasicLinePipeline>(linePipelinename,
                                                          mDevice.get(),
                                                          mRenderContext.get(),
                                                          mVkRenderPass,
                                                          basicPointLayout);
  Eigen::Matrix4f S            = Eigen::Matrix4f::Identity();
  S(0, 0)                      = 4.0f;
  S(1, 1)                      = 4.0f;
  S(2, 2)                      = 4.0f;

  mIs.push_back(S);

  mOriginAxisRenderer = std::make_unique<AxisRenderer>(1);
  mOriginAxisRenderer->setCamUB(mCameraUB.get());
  mOriginAxisRenderer->setMwcs(&mIs);
  mOriginAxisRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               axisPipeline);
}

void SLAMApp::createAxisRenderer() {
  using RP = ResourcePool;

  constexpr char linePipelinename[] = "basic_line";
  auto*          axisPipeline       = RP::requestPipeline(linePipelinename);

  mAxisRenderer = std::make_unique<AxisRenderer>(2000);
  mAxisRenderer->setCamUB(mCameraUB.get());
  mAxisRenderer->prepare(mDevice.get(),
                         mRenderContext.get(),
                         mVkDescPool,
                         mVkRenderPass,
                         axisPipeline);

  mAxisRenderer->setMwcs(&mMWcs);
}

void SLAMApp::updateSLAMData() {
  auto* info = toy::SLAMInfo::getInstance();

  if (info->getLocalPath(mMWcs)) {
    mAxisRenderer->updateSyndId();
    //VklLogD("current local path size : {}", mMWcs.size());
    Eigen::Vector3f vec = mMWcs.back().block<3, 1>(0, 3);
    VklLogD("latest frame pose : {} ", eigenVec(vec));
  }

  if (info->getLocalPoints(mLocalPointClouds)) {
    mPointCloudRenderer->updateSyncId();
  }
}

}  //namespace vkl
