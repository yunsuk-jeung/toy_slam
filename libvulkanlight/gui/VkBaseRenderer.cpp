#include <iostream>
#include "VkLogger.h"
#include "Device.h"
#include "RenderContext.h"
#include "Utils.h"
#include "ShaderModule.h"
#include "ResourcePool.h"
#include "VkBaseRenderer.h"

namespace vkl {
VkBaseRenderer::VkBaseRenderer()
  : vertShader{nullptr}
  , fragShader{nullptr}
  , subpassId{0}
  , M{Eigen::Matrix4f::Identity()}
  , R{Eigen::Matrix4f::Identity()}
  , T{Eigen::Matrix4f::Identity()}
  , S{Eigen::Matrix4f::Identity()}
  , TRS{Eigen::Matrix4f::Identity()} {}

VkBaseRenderer::~VkBaseRenderer() {
  if (!device) return;

  vk::Device vkDevice = device->getVkDevice();

  while (!createdDescriptorSetLayouts.empty()) {
    auto& descriptorSetLayout = createdDescriptorSetLayouts.top();
    device->getVkDevice().destroyDescriptorSetLayout(descriptorSetLayout);
    createdDescriptorSetLayouts.pop();
  }

  if (vkPipelineLayout) {
    vkDevice.destroyPipelineLayout(vkPipelineLayout);
    vkPipelineLayout = VK_NULL_HANDLE;
  }

  if (vkPipeline) {
    vkDevice.destroyPipeline(vkPipeline);
    vkPipeline = VK_NULL_HANDLE;
  }

  vertShader = nullptr;
  fragShader = nullptr;
}

void VkBaseRenderer::onWindowResized(int w, int h) {}

void VkBaseRenderer::initialize(Device*            _device,
                                RenderContext*     context,
                                vk::DescriptorPool descPool) {
  device           = _device;
  renderContext    = context;
  vkDescriptorPool = descPool;

  if (shaderName == "Base") { VklLogE("please set shaderName"); }

  vertShader = ResourcePool::loadShader(shaderName,
                                        device,
                                        shaderSrcType,
                                        vk::ShaderStageFlagBits::eVertex,
                                        vertShaderSource);

  fragShader = ResourcePool::loadShader(shaderName,
                                        device,
                                        shaderSrcType,
                                        vk::ShaderStageFlagBits::eFragment,
                                        fragShaderSource);
}

void VkBaseRenderer::prepare(vk::RenderPass renderPass, uint32_t _subpassId) {
  subpassId = _subpassId;
  createResource();

  createVkDescriptorSetLayout();
  createVkDescriptorSets();
  updateDescriptorSets();

  createVkPipelineLayout();
  createVkPipeline(renderPass);
}

void VkBaseRenderer::buildCommandBuffer(vk::CommandBuffer cmdBuffer, uint32_t idx) {}

void VkBaseRenderer::updateUniforms(uint32_t idx) {}

void VkBaseRenderer::setM(const Eigen::Matrix4f& in) {
  M = in;
}

void VkBaseRenderer::setT(const Eigen::Matrix4f& in) {
  T   = in;
  TRS = T * R * S;
}

void VkBaseRenderer::setR(const Eigen::Matrix4f& in) {
  R   = in;
  TRS = T * R * S;
}

void VkBaseRenderer::setS(const Eigen::Matrix4f& in) {
  S   = in;
  TRS = T * R * S;
}

void VkBaseRenderer::onRender() {}

void VkBaseRenderer::reset() {}

void VkBaseRenderer::setCamDescSetLayout(vk::DescriptorSetLayout         camLayout,
                                         std::vector<vk::DescriptorSet>* sets) {
  camDescSetLayout = camLayout;
  camDescSets      = sets;
}

void VkBaseRenderer::createResource() {
  createVertexBuffers();
  createIndexBuffers();
  createUniformBuffers();
  createTextures();
}

void VkBaseRenderer::createVkDescriptorSetLayout() {}

void VkBaseRenderer::createVkDescriptorSets() {}

void VkBaseRenderer::updateDescriptorSets() {}

void VkBaseRenderer::createVertexBuffers() {}

void VkBaseRenderer::createIndexBuffers() {}

void VkBaseRenderer::createUniformBuffers() {}

void VkBaseRenderer::createTextures() {}

}  //namespace vkl
