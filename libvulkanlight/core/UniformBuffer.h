#pragma once

#include <vector>
#include <vulkan/vulkan.hpp>
#include "Buffer.h"
#include "DescriptorSetLayout.h"
#include "Device.h"

namespace vkl {
class UniformBuffer {
public:
  USING_SMART_PTR(UniformBuffer);

  UniformBuffer() = delete;
  UniformBuffer(Device*                         device,
                std::vector<vk::DescriptorSet>& descSets,
                uint32_t                        binding,
                uint64_t                        memSize)
    : mDevice{device}
    , mVkDescSets{descSets}
    , mBinding{binding}
    , mMemSize{memSize} {
    mUBs.resize(mVkDescSets.size());

    for (auto& UB : mUBs) {
      UB = std::make_unique<Buffer>(mDevice,
                                    mMemSize,
                                    vk::BufferUsageFlagBits::eUniformBuffer,
                                    vk::MemoryPropertyFlagBits::eHostVisible,
                                    vk::MemoryPropertyFlagBits::eHostCoherent);
    }

    updateDescSets();
  }

  ~UniformBuffer() { mUBs.clear(); }

  void update(int idx, void* ptr, int offset = 0) { mUBs[idx]->update(ptr, mMemSize, 0); }

  //auto& getVkDescSets() { return mVkDescSets; }
  //auto& getVkDescSet(uint32_t idx) { return mVkDescSets[idx]; }

protected:
  virtual void updateDescSets() {
    const auto bufferCount = mVkDescSets.size();
    for (size_t i = 0; i < bufferCount; i++) {
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
  std::vector<vk::DescriptorSet> mVkDescSets;
  uint64_t                       mMemSize;
  std::vector<Buffer::Uni>       mUBs;
  uint32_t                       mBinding;

  //will be set from outside
};
}  //namespace vkl