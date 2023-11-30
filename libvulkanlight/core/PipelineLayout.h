#pragma once
#include <string>
#include <vulkan/vulkan.hpp>
#include "VkObject.h"

namespace vkl {
class ShaderModule;
class Device;
class PipelineLayout : public VkObject<vk::PipelineLayout> {
public:
  PipelineLayout() = delete;
  PipelineLayout(Device* device, std::vector<ShaderModule*>& shaderModules);

protected:
  std::vector<vk::DescriptorSetLayout> prepareDescSetLayouts();

protected:
  std::string                mName;
  Device*                    mDevice;
  std::vector<ShaderModule*> mShaderModules;
};
}  //namespace vkl