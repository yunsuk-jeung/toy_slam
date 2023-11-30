#pragma once

#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>
#include "vkltypes.h"
namespace vkl {
class Device;
class RenderContext;

class RendererBase {
public:
  RendererBase() = delete;
  RendererBase(Device*            device,
               RenderContext*     context,
               vk::DescriptorPool descPool,
               uint32_t           subPassId = 0);
  virtual ~RendererBase();
  virtual void prepare();

protected:
  virtual void setName()   = 0;
  virtual void setShader() = 0;

protected:
  std::string name;

  Device*            mDevice;
  RenderContext*     mRenderContext;
  vk::DescriptorPool mDescPool;

  ShaderSourceType mShaderSrcType;
  uint32_t         mSubpassId;

  //model matrix
  Eigen::Matrix4f mM;

  //rotation translation scale
  Eigen::Matrix4f mT;
  Eigen::Matrix4f mR;
  Eigen::Matrix4f mS;
  Eigen::Matrix4f mTRS;
};
}  //namespace vkl