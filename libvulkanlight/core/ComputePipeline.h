#pragma once
#include <string>
#include "macros.h"
#include "VkObject.h"

namespace vkl {
class Device;
class PipelineLayout;
class ComputePipeline : public VkObject<vk::Pipeline> {
public:
  USING_SMART_PTR(ComputePipeline);
  ComputePipeline() = delete;
  ComputePipeline(const std::string& name,
                  Device*            device,
                  PipelineLayout*    pipelineLayout);

  void prepare();

protected:
  std::string     mName;
  Device*         mDevice;
  PipelineLayout* mPipelineLayout;
};
}  //namespace vkl