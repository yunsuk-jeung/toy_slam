#include <iostream>
#include "VkBaseRenderer.h"
#include "Device.h"
#include "RenderContext.h"
#include "Utils.h"

namespace vkl {
VkBaseRenderer::VkBaseRenderer()
  : subpassId{0}
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

  if (vertShader) {
    vkDevice.destroyShaderModule(vertShader);
    vertShader = VK_NULL_HANDLE;
  }

  if (fragShader) {
    vkDevice.destroyShaderModule(fragShader);
    fragShader = VK_NULL_HANDLE;
  }
}

void VkBaseRenderer::onWindowResized(int w, int h) {}

void VkBaseRenderer::initialize(Device*            _device,
                                RenderContext*     context,
                                vk::DescriptorPool descPool) {
  device           = _device;
  renderContext    = context;
  vkDescriptorPool = descPool;

  auto& vkDevice = device->getVkDevice();

  switch (shaderSrcType) {
  case ShaderSourceType::STRING_FILE: {
    const std::string& shaderFolderPath = Utils::getShaderPath();
    std::string        vertFile         = shaderFolderPath + "/" + vertShaderSource;
    std::string        fragFile         = shaderFolderPath + "/" + fragShaderSource;

    vertShader = VkShaderUtil::loadShader(vkDevice, vertFile);
    fragShader = VkShaderUtil::loadShader(vkDevice, fragFile);
    break;
  }
  case ShaderSourceType::STRING:
    vertShader = VkShaderUtil::loadShader(vkDevice,
                                          vertShaderSource,
                                          vk::ShaderStageFlagBits::eVertex);

    fragShader = VkShaderUtil::loadShader(vkDevice,
                                          fragShaderSource,
                                          vk::ShaderStageFlagBits::eFragment);
    break;
  case ShaderSourceType::SPV_FILE:
    break;
  case ShaderSourceType::SPV: {
    uint32_t* src  = (uint32_t*)vertShaderSource.c_str();
    size_t    size = vertShaderSource.length();
    vertShader     = VkShaderUtil::loadShader(vkDevice, src, size);

    src        = (uint32_t*)fragShaderSource.c_str();
    size       = fragShaderSource.length();
    fragShader = VkShaderUtil::loadShader(vkDevice, src, size);

    break;
  }
  default:
    break;
  }
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
