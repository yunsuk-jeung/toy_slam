#pragma once
#include <string>
#include <vulkan/vulkan.hpp>
#include "macros.h"
#include "VkObject.h"
namespace vkl {
class ShaderModule;
class Device;
class PipelineLayout : public VkObject<vk::PipelineLayout> {
public:
  USING_SMART_PTR(PipelineLayout);
  PipelineLayout() = delete;
  PipelineLayout(Device* device, std::vector<ShaderModule*>& shaderModules);
  ~PipelineLayout();

protected:
  std::vector<vk::DescriptorSetLayout> prepareDescSetLayouts();

protected:
  std::string                mName;
  Device*                    mDevice;
  std::vector<ShaderModule*> mShaderModules;

public:
  std::vector<ShaderModule*>& getShaderModules() { return mShaderModules; }
  std::string&                getName() { return mName; }
};
}  //namespace vkl