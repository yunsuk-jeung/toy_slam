#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <string>
#include <vulkan/vulkan.hpp>

namespace vkl {
class Device;
class RenderContext;
class ShaderModule;
class PipelineLayout;
class GraphicsPipeline;
class UniformBuffer;
class RendererBase {
public:
  RendererBase();
  virtual ~RendererBase();

  virtual void onWindowResized(int w, int h);

  virtual void setM(const Eigen::Matrix4f&);
  virtual void setT(const Eigen::Matrix4f&);
  virtual void setR(const Eigen::Matrix4f&);
  virtual void setS(const Eigen::Matrix4f&);
  virtual void updateUniforms(uint32_t idx);

  virtual void prepare(Device*            device,
                       RenderContext*     context,
                       vk::DescriptorPool descPool,
                       vk::RenderPass     vkRenderPass,
                       GraphicsPipeline*  pipeline);

protected:
  virtual void createVertexBuffer();
  virtual void createIndexBuffer();
  virtual void createUniformBuffer();
  virtual void createTexture();

protected:
  std::string        mName;
  Device*            mDevice;
  RenderContext*     mRenderContext;
  vk::DescriptorPool mVkDescPool;

  vk::RenderPass    mVkRenderPass;
  PipelineLayout*   mPipelineLayout;
  GraphicsPipeline* mPipeline;

  //model matrix
  Eigen::Matrix4f mM;

  //rotation translation scale
  Eigen::Matrix4f mT;
  Eigen::Matrix4f mR;
  Eigen::Matrix4f mS;
  Eigen::Matrix4f mTRS;

public:
};
}  //namespace vkl