#pragma once
#include <string>
#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>
#include "vkltypes.h"
namespace vkl {
class Device;
class RenderContext;
class ShaderModule;
class RendererBase {
public:
  RendererBase();
  void initialize(Device*            device,
                  RenderContext*     context,
                  vk::DescriptorPool descPool,
                  uint32_t           subPassId = 0);
  virtual ~RendererBase();
  virtual void prepare();

protected:
  virtual void setName()   = 0;
  virtual void setShader() = 0;

protected:
  std::string mName;

  Device*            mDevice;
  RenderContext*     mRenderContext;
  vk::DescriptorPool mDescPool;

  ShaderSourceType mShaderSrcType;
  std::string      mShaderName{"Base"};
  std::string      mVertShaderSource{"Please set vert shader Path"};
  std::string      mFragShaderSource{"Please set frag shader Path"};
  ShaderModule*    mVertShader;
  ShaderModule*    mFragShader;

  uint32_t mSubpassId;

  //model matrix
  Eigen::Matrix4f mM;

  //rotation translation scale
  Eigen::Matrix4f mT;
  Eigen::Matrix4f mR;
  Eigen::Matrix4f mS;
  Eigen::Matrix4f mTRS;
};
}  //namespace vkl