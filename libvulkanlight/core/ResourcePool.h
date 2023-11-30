#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>

namespace vkl {
class ShaderModule;
class Device;
class ResourcePool {
public:
  static void clear();

  static ShaderModule* loadShader(Device* , uint32_t* spirv, vk::DeviceSize srcSize);

  static ShaderModule* loadShader(Device* device, std::string& filename);

  static ShaderModule*
  loadShader(Device* device, std::string& fileContents, vk::ShaderStageFlagBits stage);

protected:
  static std::unordered_map<uint64_t, std::unique_ptr<ShaderModule>> shaderPools;
};
}  //namespace vkl