#pragma once

#include <vulkan/vulkan.hpp>
#include <vector>

#include "Buffer.h"
#include "Device.h"

namespace vkl {
class UniformBuffer {
public:
  USING_SMART_PTR(UniformBuffer);
  UniformBuffer() = delete;
  UniformBuffer(Device* device_, uint32_t bufferCount_, uint64_t _memSize)
    : mDevice{device_}
    , mBufferCount{bufferCount_}
    , mMemSize{_memSize} {
    mUBs.resize(mBufferCount);
    for (auto& UB : mUBs) {
      UB = std::make_unique<Buffer>(mDevice,
                                    mMemSize,
                                    vk::BufferUsageFlagBits::eUniformBuffer,
                                    vk::MemoryPropertyFlagBits::eHostVisible,
                                    vk::MemoryPropertyFlagBits::eHostCoherent);
    }
  }
  ~UniformBuffer() {}

  void createDescSets(uint32_t                binding,
                      vk::DescriptorSetLayout descSetLayout,
                      vk::DescriptorPool      descPool) {
    mVkDescLayout = descSetLayout;
    mVkDescPool   = descPool;

    mVkDescSets.resize(mBufferCount);
    for (auto& descset : mVkDescSets) {
      descset = mDevice->vk()
                  .allocateDescriptorSets({mVkDescPool, mVkDescLayout})
                  .front();
    }

    for (size_t i = 0; i < mBufferCount; i++) {
      vk::DescriptorBufferInfo descBufferInfo(mUBs[i]->vk(), 0, mMemSize);

      vk::WriteDescriptorSet writeDescriptorSet(mVkDescSets[i],
                                                binding,
                                                {},
                                                vk::DescriptorType::eUniformBuffer,
                                                {},
                                                descBufferInfo);
      mDevice->vk().updateDescriptorSets(writeDescriptorSet, nullptr);
    }
  }

  void update(int idx, void* ptr, int offset = 0) { mUBs[idx]->update(ptr, mMemSize, 0); }

  uint32_t getBufferCount() { return mBufferCount; }
  auto&    getVkDescSets() { return mVkDescSets; }
  auto&    getVkDescLayout() { return mVkDescLayout; }

protected:
  Device*                        mDevice;
  uint64_t                       mMemSize;
  uint32_t                       mBufferCount;
  std::vector<Buffer::Uni>       mUBs;
  vk::DescriptorSetLayout        mVkDescLayout;
  std::vector<vk::DescriptorSet> mVkDescSets;

  //will be set from outside
  vk::DescriptorPool mVkDescPool;
};
}  //namespace vkl