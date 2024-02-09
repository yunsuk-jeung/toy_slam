#pragma once

#include <vector>
#include <vulkan/vulkan.hpp>

#include "Buffer.h"
#include "BufferingBuffer.h"
#include "Device.h"

namespace vkl {
class DynamicUniformBuffer : protected BufferingBuffer {
public:
  DynamicUniformBuffer() = delete;
  DynamicUniformBuffer(Device*                        device,
                       std::vector<vk::DescriptorSet> descSets,
                       uint32_t                       binding,
                       uint64_t                       totalSize,
                       uint32_t                       uniformSize,
                       uint32_t                       alignment)
    : BufferingBuffer(device,
                      descSets.size(),
                      totalSize,
                      vk::BufferUsageFlagBits::eUniformBuffer,
                      vk::MemoryPropertyFlagBits::eHostVisible,
                      vk::MemoryPropertyFlagBits::eHostCoherent)
    , mVkDescSets{descSets}
    , mBinding{binding}
    , mUniformSize{uniformSize}
    , mAlignment{alignment} {
    createDescSets();
  }

  auto& getVkDescSet(uint32_t idx) { return mVkDescSets[idx]; }
  void  update(uint32_t idx, size_t count, size_t dataAlignment, void* data) {
    uint8_t* dst = mBuffers[idx]->map();
    uint8_t* src = (uint8_t*)data;

    for (int i = 0; i < count; i++) {
      memcpy(dst, src, mUniformSize);
      dst += mAlignment;
      src += dataAlignment;
    }
  }

  uint32_t& getAlignment() { return mAlignment; }

protected:
  virtual void createDescSets() {
    for (size_t i = 0; i < mBufferCount; i++) {
      vk::DescriptorBufferInfo descBufferInfo(mBuffers[i]->vk(), 0, mUniformSize);

      vk::WriteDescriptorSet writeDescriptorSet(mVkDescSets[i],
                                                mBinding,
                                                {},
                                                vk::DescriptorType::eUniformBufferDynamic,
                                                {},
                                                descBufferInfo);
      mDevice->vk().updateDescriptorSets(writeDescriptorSet, nullptr);
    }
  }

protected:
  const std::vector<vk::DescriptorSet> mVkDescSets;
  uint32_t                             mBinding;
  uint32_t                             mUniformSize;
  uint32_t                             mAlignment;
};
}  //namespace vkl