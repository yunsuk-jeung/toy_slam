#pragma once
#include <map>
#include <string>
#include <vulkan/vulkan.hpp>
#include "VkObject.h"
#include "macros.h"

namespace vkl {
class ShaderModule;
class Device;
class DescriptorSetLayout;
class PipelineLayout : public VkObject<vk::PipelineLayout> {
public:
  USING_SMART_PTR(PipelineLayout);
  PipelineLayout() = delete;
  PipelineLayout(Device* device, std::vector<ShaderModule*>& shaderModules);
  ~PipelineLayout();

  DescriptorSetLayout* getDescriptorSetLayout(const uint32_t& setId);

protected:
  std::vector<vk::DescriptorSetLayout> prepareDescSetLayouts();

protected:
  std::string                     mName;
  Device*                         mDevice;
  std::vector<ShaderModule*>      mShaderModules;
  std::map<uint32_t, std::string> mDescsetNameMap;

public:
  std::vector<ShaderModule*>& getShaderModules() { return mShaderModules; }
  std::string&                getName() { return mName; }
};
}  //namespace vkl