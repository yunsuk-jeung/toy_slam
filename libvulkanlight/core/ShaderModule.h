#pragma once
#include "VKObject.h"
#include "Device.h"

namespace vkl {
class ShaderModule : public VkObject<vk::ShaderModule> {
public:
  USING_SMART_PTR(ShaderModule);

  ShaderModule() = delete;
  ShaderModule(Device* device)
    : mDevice{device} {}
  ~ShaderModule() { mDevice->getVkDevice().destroyShaderModule(this->vk()); }

protected:
  Device*               mDevice;
  std::vector<uint32_t> mSpirv;

public:
  void setSpirv(const std::vector<uint32_t>& spirv) { mSpirv = spirv; }
};
}  //namespace vkl