#include "BaseRenderer.h"

namespace vkl {
BaseRenderer::BaseRenderer()
  : name{"Base Renderer"}
  , mSubpassId{0}
  , mM{Eigen::Matrix4f::Identity()}
  , mR{Eigen::Matrix4f::Identity()}
  , mT{Eigen::Matrix4f::Identity()}
  , mS{Eigen::Matrix4f::Identity()}
  , mTRS{Eigen::Matrix4f::Identity()} {}

BaseRenderer::~BaseRenderer() {}
}  //namespace vkl
