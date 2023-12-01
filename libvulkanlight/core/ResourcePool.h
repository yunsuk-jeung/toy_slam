#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "vkltypes.h"

namespace vkl {
class Device;
class PipelineLayout;
class ShaderModule;
class ResourcePool {
public:
  static void clear(Device* device);

  static ShaderModule* loadShader(const std::string&      name,
                                  Device*                 device,
                                  ShaderSourceType        srcType,
                                  vk::ShaderStageFlagBits stage,
                                  const std::string&      shaderSrc);

  static void addDescriptorSetLayouts(const std::string&      name,
                                      vk::DescriptorSetLayout descSetLayout);

  static void addPipelineLayout(Device* device, ShaderModule* vert, ShaderModule* frag);

  static PipelineLayout* requestPipelineLayout(const std::string& name);

  static void addPipeline(const std::string& name, vk::Pipeline pipeline);

  static vk::Pipeline requestPipeline(const std::string& name);

protected:

protected:
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>> mShaderPools;
  static std::unordered_map<size_t, vk::DescriptorSetLayout>       mDescriptorSetLayouts;
  static std::unordered_map<size_t, std::unique_ptr<PipelineLayout>> mPipelineLayouts;
  static std::unordered_map<size_t, vk::Pipeline>                    mPipelines;
};
}  //namespace vkl
