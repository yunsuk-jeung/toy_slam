#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "ResourcePool.h"
#include "RendererBase.h"
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
  , mTRS{Eigen::Matrix4f::Identity()}
  , mCamUB{nullptr} {}

RendererBase::~RendererBase() {}

void RendererBase::onWindowResized(int w, int h) {}

void RendererBase::prepare(Device*            device,
                           RenderContext*     context,
                           vk::DescriptorPool descPool,
                           vk::RenderPass     renderPass,
                           Pipeline*          pipeline) {
  mDevice        = device;
  mRenderContext = context;
  mVkDescPool    = descPool;
  mVkRenderPass  = renderPass;

  mPipeline       = pipeline;
  mPipelineLayout = mPipeline->getPipelineLayout();

  createVertexBuffer();
  createIndexBuffers();
  createUniformBuffers();
  createTextures();
  createDescriptorsets();
  updateDescriptorsets();
}

void RendererBase::createVertexBuffer() {}
void RendererBase::createIndexBuffers() {}
void RendererBase::createUniformBuffers() {}
void RendererBase::createTextures() {}
void RendererBase::createDescriptorsets() {}
void RendererBase::updateDescriptorsets() {}

}  //namespace vkl
