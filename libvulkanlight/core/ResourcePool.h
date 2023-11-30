#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "vkltypes.h"

namespace vkl {
class ShaderModule;
class Device;
class ResourcePool {
public:
  static void clear();

  static ShaderModule*
  loadShader(Device* device, ShaderSourceType srcType, std::string& src);

protected:
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>> shaderPools;
};
}  //namespace vkl