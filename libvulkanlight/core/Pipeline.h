#pragma once
#include "VkObject.h"

namespace vkl {
class Device;
class PipelineLayout;
class Pipeline : public VkObject<vk::Pipeline> {
public:
  Pipeline() = delete;
  Pipeline(Device* device, vk::RenderPass renderPass, PipelineLayout* pipelineLayout);
  virtual ~Pipeline();

protected:
  Device*         mDevice;
  vk::RenderPass  mRenderPass;
  PipelineLayout* mPipelineLayout;
};

}  //namespace vkl