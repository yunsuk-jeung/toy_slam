#pragma once
#include "Object.h"

namespace vkl {
class Pipeline : Object<vk::Pipeline> {
public:
  Pipeline()  = default;
  ~Pipeline() = default;

protected:
  vk::PipelineLayout      mPipelineLayout;
  vk::DescriptorSetLayout mDescSetLayouts;
}
}  //namespace vkl