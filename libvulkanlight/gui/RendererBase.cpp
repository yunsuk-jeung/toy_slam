#include "ShaderModule.h"
#include "ResourcePool.h"
#include "RendererBase.h"
namespace vkl {
RendererBase::RendererBase()
  : mName{"Base Renderer"}
  , mShaderSrcType{ShaderSourceType::STRING}
  , mM{Eigen::Matrix4f::Identity()}
  , mR{Eigen::Matrix4f::Identity()}
  , mT{Eigen::Matrix4f::Identity()}
  , mS{Eigen::Matrix4f::Identity()}
  , mTRS{Eigen::Matrix4f::Identity()} {}

void RendererBase::initialize(Device*            device,
                              RenderContext*     context,
                              vk::DescriptorPool descPool,
                              uint32_t           subPassId) {
  mDevice        = device;
  mRenderContext = context;
  mDescPool      = descPool;
  mSubpassId     = subPassId;
  setName();
  setShader();

  mVertShader = ResourcePool::loadShader(mShaderName,
                                         mDevice,
                                         mShaderSrcType,
                                         vk::ShaderStageFlagBits::eVertex,
                                         mVertShaderSource);
}

RendererBase::~RendererBase() {}

void RendererBase::prepare() {}
}  //namespace vkl
