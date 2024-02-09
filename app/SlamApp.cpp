#include <iostream>
#include <string>
#include <iomanip>
#include "Simulator.h"
#include "Window.h"
#include "GraphicsCamera.h"
#include "GUI.h"
#include "Device.h"
#include "VklShaderUtil.h"
#include "ShaderModule.h"
#include "ResourcePool.h"
#include "AxisRenderer.h"
#include "PointCloudRenderer.h"
#include "GraphicsPipeline.h"
#include "VklLogger.h"
#include "GuiObject.h"
#include "SLAMInfo.h"
#include "SLAMApp.h"

namespace vkl {
namespace {
enum eSHADERS {
  BASIC_POINT = 0,
  BASIC       = 1,

};
enum ePIPELINES { BASIC_POINT_PL, BASIC_LINE_PL };

constexpr char* SHADERS[]   = {"basicPoint", "basic"};
constexpr char* PIPELINES[] = {"basic_point", "basic_line"};

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
  vklLogD("app directory : {}", dir);

  addShaderPath(dir);
  addAssetPath(dir);

  mContinousMode = true;
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

  Eigen::Quaternionf Qyz = Eigen::Quaternionf::FromTwoVectors(-Eigen::Vector3f::UnitY(),
                                                              Eigen::Vector3f::UnitZ());
  mGraphicsCamera->cam.M.block<3, 3>(0, 0) = Qyz.toRotationMatrix();

  auto* simulator = (io::Simulator*)mSensor;
  simulator->setContinuosMode(true);

  GuiImpl::RenderImpl impl = [this, simulator]() {
    if (ImGui::Button("change mode")) {
      simulator->changeContinousMode();
    }
    ImGui::SameLine();

    if (simulator->getContinuousMode()) {
      ImGui::Text("mode : conitnuous");
    }
    else {
      ImGui::Text("mode : one time submit");
    }

    if (ImGui::Button("next image")) {
      ((io::Simulator*)mSensor)->sendImage();
    }
  };

  GuiImpl::Ptr obj = std::make_shared<GuiImpl>(impl);
  mGUI->addGuiImpls(obj);

  mSlamKeyCallback = std::make_unique<InputCallback>();
  auto keyCallback = [&](int key) {
    switch (key) {
    case 524: {
      ((io::Simulator*)mSensor)->sendImage();
      break;
    }
    default:
      break;
    }
  };

  mSlamKeyCallback->registerKeyPressed(std::move(keyCallback));

  mGUI->addInputCallback(mSlamKeyCallback.get());

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

void SLAMApp::createPipelines() {
  {
    ShaderSourceType type = ShaderSourceType::STRING_FILE;
    constexpr char*  vert = SHADERS[BASIC_POINT];
    constexpr char*  frag = SHADERS[BASIC];
    constexpr char*  PLN  = PIPELINES[BASIC_POINT_PL];

    using RP         = ResourcePool;
    auto* vertShader = RP::loadShader(vert, type, vk::ShaderStageFlagBits::eVertex);
    auto* fragShader = RP::loadShader(frag, type, vk::ShaderStageFlagBits::eFragment);
    auto* PLL        = RP::addPipelineLayout(vertShader, fragShader);
    auto* PL = RP::addGraphicsPipeline(PLN, mRenderContext.get(), mVkRenderPass, PLL);

    std::vector<vk::VertexInputBindingDescription> bd{
      {0, sizeof(Vertex4d), vk::VertexInputRate::eVertex}
    };

    std::vector<vk::VertexInputAttributeDescription> ad{
      {0, 0, vk::Format::eR32G32B32A32Sfloat, 0},
    };
    PL->setVertexDescription(bd, ad);
    PL->setPrimitiveTopology(vk::PrimitiveTopology::ePointList);
    PL->prepare();
  }
  {
    ShaderSourceType type = ShaderSourceType::STRING_FILE;
    constexpr char*  vert = SHADERS[BASIC];
    constexpr char*  frag = SHADERS[BASIC];
    constexpr char*  PLN  = PIPELINES[BASIC_LINE_PL];

    using RP         = ResourcePool;
    auto* vertShader = RP::loadShader(vert, type, vk::ShaderStageFlagBits::eVertex);
    auto* fragShader = RP::loadShader(frag, type, vk::ShaderStageFlagBits::eFragment);
    auto& shaderResources = vertShader->getShaderResources();

    for (auto& resource : shaderResources) {
      if (resource.set != 1) {
        continue;
      }
      resource.mode = ShaderResourceMode::Dynamic;
    }
    auto* PLL = RP::addPipelineLayout(vertShader, fragShader);
    auto* PL  = RP::addGraphicsPipeline(PLN, mRenderContext.get(), mVkRenderPass, PLL);
    std::vector<vk::VertexInputBindingDescription> bd{
      {0, sizeof(VertexColor), vk::VertexInputRate::eVertex}
    };

    std::vector<vk::VertexInputAttributeDescription> ad{
      {0, 0, vk::Format::eR32G32B32Sfloat, 0},
      {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexColor, r)},
    };
    PL->setVertexDescription(bd, ad);
    PL->setPrimitiveTopology(vk::PrimitiveTopology::eLineList);
    auto& RCI = PL->getRasterizationStateCI();
    RCI.lineWidth = 3.0f;
    PL->prepare();
  }
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

  auto& idx = mCurrBufferingIdx;

  mOriginAxisRenderer->buildCommandBuffer(cmd, idx, mCameraDescriptor.descSets[idx]);
  mPointCloudRenderer->buildCommandBuffer(cmd, idx, mCameraDescriptor.descSets[idx]);

  mAxisRenderer->buildCommandBuffer(cmd, idx, mCameraDescriptor.descSets[idx]);

  if (mGUI)
    mGUI->buildCommandBuffer(cmd, idx);

  cmd.endRenderPass();
  cmd.end();
}

void SLAMApp::updateUniform(int idx) {
  updateCameraUniform(idx);
}

void SLAMApp::createPointRenderer() {
  auto* PL            = ResourcePool::requestGraphicsPipeline(PIPELINES[BASIC_POINT_PL]);
  mPointCloudRenderer = std::make_unique<PointCloudRenderer>();
  mPointCloudRenderer->setPointCloud(&mLocalPointClouds);
  mPointCloudRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               PL);
}

void SLAMApp::createOriginRenderer() {
  Eigen::Matrix4f S = Eigen::Matrix4f::Identity();
  S(0, 0)           = 4.0f;
  S(1, 1)           = 4.0f;
  S(2, 2)           = 4.0f;

  mIs.push_back(S);

  auto* PL            = ResourcePool::requestGraphicsPipeline(PIPELINES[BASIC_LINE_PL]);
  mOriginAxisRenderer = std::make_unique<AxisRenderer>(1);
  mOriginAxisRenderer->setMwcs(&mIs);
  mOriginAxisRenderer->prepare(mDevice.get(),
                               mRenderContext.get(),
                               mVkDescPool,
                               mVkRenderPass,
                               PL);
}

void SLAMApp::createAxisRenderer() {
  auto* PL = ResourcePool::requestGraphicsPipeline(PIPELINES[BASIC_LINE_PL]);

  mAxisRenderer = std::make_unique<AxisRenderer>(2000);
  mAxisRenderer->setMwcs(&mMWcs);
  mAxisRenderer->prepare(mDevice.get(),
                         mRenderContext.get(),
                         mVkDescPool,
                         mVkRenderPass,
                         PL);
}

void SLAMApp::updateSLAMData() {
  auto* info = toy::SLAMInfo::getInstance();

  if (info->getLocalPath(mMWcs)) {
    mAxisRenderer->updateSyndId();
    //vklLogD("current local path size : {}", mMWcs.size());
    //Eigen::Vector3f vec = mMWcs.back().block<3, 1>(0, 3);
    //vklLogD("latest frame pose : {} ", eigenVec(vec));
  }

  if (info->getLocalPoints(mLocalPointClouds)) {
    mPointCloudRenderer->updateSyncId();
  }
}

}  //namespace vkl
