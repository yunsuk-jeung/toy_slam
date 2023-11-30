#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "vkltypes.h"
#include "ShaderModule.h"

namespace vkl {
class Device;
class ResourcePool {
public:
  static void clear();

  static ShaderModule* loadShader(const std::string&      name,
                                  Device*                 device,
                                  ShaderSourceType        srcType,
                                  vk::ShaderStageFlagBits stage,
                                  std::string&            shaderSrc);

  static void addDescriptorSetLayouts(const std::string&      name,
                                      vk::DescriptorSetLayout descSetLayout);

protected:

protected:
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>> mShaderPools;
  //static std::unordered_map<size_t, std::unique_ptr<>> mShaderPools;
  static std::unordered_map<size_t, vk::DescriptorSetLayout>       mDescriptorSetLayouts;
};
}  //namespace vkl
