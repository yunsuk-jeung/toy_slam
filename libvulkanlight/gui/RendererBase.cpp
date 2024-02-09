#include "RendererBase.h"
#include "GraphicsPipeline.h"
#include "PipelineLayout.h"
#include "ResourcePool.h"
#include "ShaderModule.h"

namespace vkl {
RendererBase::RendererBase()
  : mName{"Base Renderer"}
  , mDevice{nullptr}
  , mRenderContext{nullptr}
  , mVkDescPool{VK_NULL_HANDLE}
  , mVkRenderPass{VK_NULL_HANDLE}
  , mPipelineLayout{nullptr}
  , mPipeline{nullptr}
  , mM{Eigen::Matrix4f::Identity()}
  , mR{Eigen::Matrix4f::Identity()}
  , mT{Eigen::Matrix4f::Identity()}
  , mS{Eigen::Matrix4f::Identity()}
  , mTRS{Eigen::Matrix4f::Identity()} {}

RendererBase::~RendererBase() {}

void RendererBase::onWindowResized(int w, int h) {}

void RendererBase::setM(const Eigen::Matrix4f& in) {
  mM = in;
}

void RendererBase::setT(const Eigen::Matrix4f& in) {
  mT   = in;
  mTRS = mT * mR * mS;
}

void RendererBase::setR(const Eigen::Matrix4f& in) {
  mR   = in;
  mTRS = mT * mR * mS;
}

void RendererBase::setS(const Eigen::Matrix4f& in) {
  mS   = in;
  mTRS = mT * mR * mS;
}

void RendererBase::updateUniforms(uint32_t idx) {}

void RendererBase::prepare(Device*            device,
                           RenderContext*     context,
                           vk::DescriptorPool descPool,
                           vk::RenderPass     renderPass,
                           GraphicsPipeline*  pipeline) {
  mDevice        = device;
  mRenderContext = context;
  mVkDescPool    = descPool;
  mVkRenderPass  = renderPass;

  mPipeline       = pipeline;
  mPipelineLayout = mPipeline->getPipelineLayout();

  createVertexBuffer();
  createIndexBuffer();
  createUniformBuffer();
  createTexture();
}

void RendererBase::createVertexBuffer() {}
void RendererBase::createIndexBuffer() {}
void RendererBase::createUniformBuffer() {}
void RendererBase::createTexture() {}

}  //namespace vkl
