#pragma once

#include <map>
#include <string>
#include <vulkan/vulkan.hpp>

namespace vkl {
class ShaderPool {
public:
  ShaderPool()  = default;
  ~ShaderPool() = default;

protected:
  static std::map<int, vk::ShaderModule> mVertexShaders;
  static std::map<int, vk::ShaderModule> mFragmentShaders;
  static vk::ShaderModule                requestShaderModule();
}
}  //namespace vkl