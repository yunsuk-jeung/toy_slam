#pragma once

#include <stack>
#include <Eigen/Dense>
#include "vkltypes.h"
#include "VkShaderUtil.h"

namespace vkl {

class Device;
class RenderContext;
class Buffer;
class VkBaseRenderer {
public:
  VkBaseRenderer();
  virtual ~VkBaseRenderer();
  virtual void onWindowResized(int w, int h);

  virtual void
  initialize(Device* device, RenderContext* context, vk::DescriptorPool descPool);
  virtual void prepare(vk::RenderPass renderPass, uint32_t subpassId = 0);
  virtual void buildCommandBuffer(vk::CommandBuffer cmdBuffer, uint32_t idx);
  virtual void updateUniforms(uint32_t idx = 0);

  virtual void setM(const Eigen::Matrix4f&);
  virtual void setT(const Eigen::Matrix4f&);
  virtual void setR(const Eigen::Matrix4f&);
  virtual void setS(const Eigen::Matrix4f&);

  virtual void onRender();
  virtual void reset();
  void         setCamDescSetLayout(vk::DescriptorSetLayout         camLayout,
                                   std::vector<vk::DescriptorSet>* sets);

protected:
  virtual void setName()   = 0;
  virtual void setShader() = 0;

  virtual void createResource();
  virtual void createVertexBuffers();
  virtual void createIndexBuffers();
  virtual void createUniformBuffers();
  virtual void createTextures();

  virtual void createVkDescriptorSetLayout();
  virtual void createVkDescriptorSets();
  virtual void updateDescriptorSets();

  virtual void createVkPipelineLayout()                    = 0;
  virtual void createVkPipeline(vk::RenderPass renderPass) = 0;

public:

protected:
  std::string      name{"Basic Renderer"};
  ShaderSourceType shaderSrcType;
  std::string      vertShaderSource{"Please set vert shader Path"};
  std::string      fragShaderSource{"Please set frag shader Path"};

  Device*        device        = nullptr;
  RenderContext* renderContext = nullptr;

  vk::ShaderModule vertShader = VK_NULL_HANDLE;
  vk::ShaderModule fragShader = VK_NULL_HANDLE;

  uint32_t subpassId;

  vk::PipelineLayout vkPipelineLayout = VK_NULL_HANDLE;
  vk::Pipeline       vkPipeline       = VK_NULL_HANDLE;

  vk::DescriptorPool vkDescriptorPool = VK_NULL_HANDLE;

  std::stack<vk::DescriptorSetLayout> createdDescriptorSetLayouts = {};

  vk::DescriptorSetLayout         camDescSetLayout = VK_NULL_HANDLE;
  std::vector<vk::DescriptorSet>* camDescSets      = {};

  //model matrix
  Eigen::Matrix4f M;

  //rotation translation scale
  Eigen::Matrix4f T;
  Eigen::Matrix4f R;
  Eigen::Matrix4f S;
  Eigen::Matrix4f TRS;
};
}  //namespace vkl
