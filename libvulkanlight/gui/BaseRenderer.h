#pragma once

#include <vulkan/vulkan.hpp>
#include <Eigen/Dense>

namespace vkl {
class BaseRenderer {
public:
  BaseRenderer();
  virtual ~BaseRenderer();

protected:
  std::string name{};

  uint32_t     mSubpassId;
  vk::Pipeline mPipeline;

  //model matrix
  Eigen::Matrix4f mM;

  //rotation translation scale
  Eigen::Matrix4f mT;
  Eigen::Matrix4f mR;
  Eigen::Matrix4f mS;
  Eigen::Matrix4f mTRS;
};
}  //namespace vkl