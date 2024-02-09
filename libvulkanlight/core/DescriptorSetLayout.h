#pragma once
#include "VkObject.h"
#include "Device.h"

namespace vkl {
class DescriptorSetLayout : public VkObject<vk::DescriptorSetLayout> {
public:
  DescriptorSetLayout() = delete;
  DescriptorSetLayout(Device*                                            device,
                      const std::vector<vk::DescriptorSetLayoutBinding>& bindings)
    : mDevice(device)
    , mBindings{bindings} {
    mVkObject = mDevice->vk().createDescriptorSetLayout({{}, bindings});
  }
  ~DescriptorSetLayout() { mDevice->vk().destroyDescriptorSetLayout(mVkObject); }

protected:
  Device*                                     mDevice;
  std::vector<vk::DescriptorSetLayoutBinding> mBindings;

public:
  std::vector<vk::DescriptorSetLayoutBinding>& getBindings() { return mBindings; }
};
}  //namespace vkl