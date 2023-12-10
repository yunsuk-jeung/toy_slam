#pragma once

#include <vulkan/vulkan.hpp>
#include <vector>

#include "Buffer.h"
#include "Device.h"
#include "BufferingBuffer.h"

namespace vkl {
class DynamicUniformBuffer : protected BufferingBuffer {
public:
  DynamicUniformBuffer() = delete;
  DynamicUniformBuffer(Device*                 device,
                       uint32_t                bufferCount,
                       uint64_t                size,
                       vk::DescriptorPool      descPool,
                       vk::DescriptorSetLayout layout,
                       uint32_t                binding,
                       uint32_t                uniformSize,
                       uint32_t                alignment)
    : BufferingBuffer(device,
                      bufferCount,
                      size,
                      vk::BufferUsageFlagBits::eUniformBuffer,
                      vk::MemoryPropertyFlagBits::eHostVisible,
                      vk::MemoryPropertyFlagBits::eHostCoherent)
    , mVkDescPool{descPool}
    , mVkDescLayout{layout}
    , mBinding{binding}
    , mUniformMemorySize{uniformSize}
    , mAlignment{alignment} {
    createDescSets();
  }

  auto& getVkDescSet(uint32_t idx) { return mVkDescSets[idx]; }
  void  update(uint32_t idx, size_t count, size_t dataAlignment, void* data) {
    uint8_t* dst = mBuffers[idx]->map();
    uint8_t* src = (uint8_t*)data;

    for (int i = 0; i < count; i++) {
      memcpy(dst, src, mUniformMemorySize);
      dst += mAlignment;
      src += dataAlignment;
    }
  }

  uint32_t& getAlignment() { return mAlignment; }

protected:
  virtual void createDescSets() {
    mVkDescSets.resize(mBufferCount);
    for (auto& descset : mVkDescSets) {
      descset = mDevice->vk()
                  .allocateDescriptorSets({mVkDescPool, mVkDescLayout})
                  .front();
    }

    for (size_t i = 0; i < mBufferCount; i++) {
      vk::DescriptorBufferInfo descBufferInfo(mBuffers[i]->vk(), 0, mUniformMemorySize);

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
  vk::DescriptorSetLayout        mVkDescLayout;
  std::vector<vk::DescriptorSet> mVkDescSets;
  uint32_t                       mBinding;

  vk::DescriptorPool mVkDescPool;

  uint32_t mUniformMemorySize;
  uint32_t mAlignment;
};
}  //namespace vkl