#pragma once
#include <string>
#include <map>
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

  vk::DescriptorSetLayout getDescriptorSetLayout(const std::string& name);

protected:
  std::vector<vk::DescriptorSetLayout> prepareDescSetLayouts();

protected:
  std::string                        mName;
  Device*                            mDevice;
  std::vector<ShaderModule*>         mShaderModules;
  std::map<std::string, std::string> mDescsetNameMap;

public:
  std::vector<ShaderModule*>& getShaderModules() { return mShaderModules; }
  std::string&                getName() { return mName; }
};
}  //namespace vkl