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
    , mMemSize{_memSize}
    , mBinding{0} {
    mUBs.resize(mBufferCount);
    for (auto& UB : mUBs) {
      UB = Buffer::Uni(new Buffer(mDevice,
                                  mMemSize,
                                  vk::BufferUsageFlagBits::eUniformBuffer,
                                  vk::MemoryPropertyFlagBits::eHostVisible,
                                  vk::MemoryPropertyFlagBits::eHostCoherent));
    }
  }
  ~UniformBuffer() {
    if (mVkDescLayout) {
      mDevice->getVkDevice().destroyDescriptorSetLayout(mVkDescLayout);
    }
  }

  void createDescSets(vk::DescriptorSetLayoutBinding descSetBinding,
                      vk::DescriptorPool             descPool) {
    mVkDescLayout = mDevice->getVkDevice().createDescriptorSetLayout(
      {{}, descSetBinding});
    mBinding    = descSetBinding.binding;
    mVkDescPool = descPool;

    mVkDescSets.resize(mBufferCount);
    for (auto& descset : mVkDescSets) {
      descset = mDevice->getVkDevice()
                  .allocateDescriptorSets({mVkDescPool, mVkDescLayout})
                  .front();
    }

    for (size_t i = 0; i < mBufferCount; i++) {
      vk::DescriptorBufferInfo descBufferInfo(mUBs[i]->getVkObject(), 0, mMemSize);

      vk::WriteDescriptorSet writeDescriptorSet(mVkDescSets[i],
                                                mBinding,
                                                {},
                                                vk::DescriptorType::eUniformBuffer,
                                                {},
                                                descBufferInfo);
      mDevice->getVkDevice().updateDescriptorSets(writeDescriptorSet, nullptr);
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
  uint32_t                       mBinding;
  std::vector<Buffer::Uni>       mUBs;
  vk::DescriptorSetLayout        mVkDescLayout;
  std::vector<vk::DescriptorSet> mVkDescSets;

  //will be set from outside
  vk::DescriptorPool mVkDescPool;
};
}  //namespace vkl