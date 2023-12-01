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
  , mDescPool{VK_NULL_HANDLE}
  , mVkRenderPass{VK_NULL_HANDLE}
  , mVkPipeline{VK_NULL_HANDLE}
  , mM{Eigen::Matrix4f::Identity()}
  , mR{Eigen::Matrix4f::Identity()}
  , mT{Eigen::Matrix4f::Identity()}
  , mS{Eigen::Matrix4f::Identity()}
  , mTRS{Eigen::Matrix4f::Identity()} {}

void RendererBase::prepare(Device*            device,
                           RenderContext*     context,
                           vk::DescriptorPool descPool,
                           vk::RenderPass     renderPass,
                           std::string        pipelineName) {
  setName();

  mDevice        = device;
  mRenderContext = context;
  mDescPool      = descPool;
  mVkRenderPass  = renderPass;
  mVkPipeline    = ResourcePool::requestPipeline(pipelineName)->vk();

  createVertexBuffer();
  createIndexBuffers();
  createUniformBuffers();
  createTextures();
  createDescriptorsets();
}

RendererBase::~RendererBase() {}

void RendererBase::createVertexBuffer() {}
void RendererBase::createIndexBuffers() {}
void RendererBase::createUniformBuffers() {}
void RendererBase::createTextures() {}
void RendererBase::createDescriptorsets() {}
}  //namespace vkl
