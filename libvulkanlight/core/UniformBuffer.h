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
  UniformBuffer(Device*                 device,
                vk::DescriptorPool      descPool,
                vk::DescriptorSetLayout layout,
                uint32_t                binding,
                uint32_t                bufferCount,
                uint64_t                memSize)
    : mDevice{device}
    , mVkDescPool{descPool}
    , mVkDescLayout{layout}
    , mBinding{binding}
    , mBufferCount{bufferCount}
    , mMemSize{memSize} {
    mUBs.resize(mBufferCount);

    for (auto& UB : mUBs) {
      UB = std::make_unique<Buffer>(mDevice,
                                    mMemSize,
                                    vk::BufferUsageFlagBits::eUniformBuffer,
                                    vk::MemoryPropertyFlagBits::eHostVisible,
                                    vk::MemoryPropertyFlagBits::eHostCoherent);
    }

    createDescSets();
  }

  ~UniformBuffer() { mUBs.clear(); }

  void update(int idx, void* ptr, int offset = 0) { mUBs[idx]->update(ptr, mMemSize, 0); }

  const uint32_t getBufferCount() const { return mBufferCount; }
  auto&          getVkDescLayout() { return mVkDescLayout; }
  auto&          getVkDescSets() { return mVkDescSets; }
  auto&          getVkDescSet(uint32_t idx) { return mVkDescSets[idx]; }

protected:
  virtual void createDescSets() {
    mVkDescSets.resize(mBufferCount);
    for (auto& descset : mVkDescSets) {
      descset = mDevice->vk()
                  .allocateDescriptorSets({mVkDescPool, mVkDescLayout})
                  .front();
    }

    for (size_t i = 0; i < mBufferCount; i++) {
      vk::DescriptorBufferInfo descBufferInfo(mUBs[i]->vk(), 0, mMemSize);

      vk::WriteDescriptorSet writeDescriptorSet(mVkDescSets[i],
                                                mBinding,
                                                {},
                                                vk::DescriptorType::eUniformBuffer,
                                                {},
                                                descBufferInfo);
      mDevice->vk().updateDescriptorSets(writeDescriptorSet, nullptr);
    }
  }

protected:
  Device*                        mDevice;
  uint64_t                       mMemSize;
  uint32_t                       mBufferCount;
  std::vector<Buffer::Uni>       mUBs;
  vk::DescriptorSetLayout        mVkDescLayout;
  std::vector<vk::DescriptorSet> mVkDescSets;
  uint32_t                       mBinding;

  //will be set from outside
  vk::DescriptorPool mVkDescPool;
};
}  //namespace vkl