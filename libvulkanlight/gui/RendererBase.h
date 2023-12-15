#pragma once
#include <string>
#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>
#include "vkltypes.h"
namespace vkl {
class Device;
class RenderContext;
class ShaderModule;
class PipelineLayout;
class Pipeline;
class UniformBuffer;
class RendererBase {
public:
  RendererBase();
  virtual ~RendererBase();

  virtual void onWindowResized(int w, int h);

  virtual void prepare(Device*            device,
                       RenderContext*     context,
                       vk::DescriptorPool descPool,
                       vk::RenderPass     vkRenderPass,
                       Pipeline*          pipeline);

protected:
  virtual void createVertexBuffer();
  virtual void createIndexBuffers();
  virtual void createUniformBuffers();
  virtual void createTextures();
  virtual void createDescriptorsets();
  virtual void updateDescriptorsets();

protected:
  std::string        mName;
  Device*            mDevice;
  RenderContext*     mRenderContext;
  vk::DescriptorPool mVkDescPool;

  vk::RenderPass  mVkRenderPass;
  PipelineLayout* mPipelineLayout;
  Pipeline*       mPipeline;

  //model matrix
  Eigen::Matrix4f mM;

  //rotation translation scale
  Eigen::Matrix4f mT;
  Eigen::Matrix4f mR;
  Eigen::Matrix4f mS;
  Eigen::Matrix4f mTRS;

  UniformBuffer* mCamUB;

public:
  void setCamUB(UniformBuffer* buffer) { mCamUB = buffer; }
};
}  //namespace vkl