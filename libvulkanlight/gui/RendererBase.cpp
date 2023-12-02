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

RendererBase::~RendererBase() {}

void RendererBase::onWindowResized(int w, int h) {}

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

  auto* pipeline    = ResourcePool::requestPipeline(pipelineName);
  mVkPipeline       = pipeline->vk();
  mVkPipelineLayout = pipeline->getPipelineLayout()->vk();

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
