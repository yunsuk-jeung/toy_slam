#include "RendererBase.h"

namespace vkl {
RendererBase::RendererBase(Device*            device,
                           RenderContext*     context,
                           vk::DescriptorPool descPool,
                           uint32_t           subPassId)
  : name{"Base Renderer"}
  , mDevice{device}
  , mRenderContext{context}
  , mDescPool{descPool}
  , mSubpassId{subPassId}
  , mM{Eigen::Matrix4f::Identity()}
  , mR{Eigen::Matrix4f::Identity()}
  , mT{Eigen::Matrix4f::Identity()}
  , mS{Eigen::Matrix4f::Identity()}
  , mTRS{Eigen::Matrix4f::Identity()} {}

RendererBase::~RendererBase() {}

void RendererBase::prepare() {}
}  //namespace vkl
