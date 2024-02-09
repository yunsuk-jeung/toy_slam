#pragma once

#include "VkObject.h"
#include <memory>
#include <vk_mem_alloc.h>

namespace vkl {

class Device;
class Buffer : public VkObject<vk::Buffer> {
public:
  USING_SMART_PTR(Buffer);
  DELETE_COPY_CONSTRUCTORS(Buffer);

  Buffer() = default;
  ~Buffer();

  explicit Buffer(Device*                      device,
                  vk::DeviceSize               size,
                  vk::BufferUsageFlags         bufferUsage,
                  vk::MemoryPropertyFlags      required,
                  vk::MemoryPropertyFlags      prefered,
                  VmaAllocationCreateFlagBits  flags = VMA_ALLOCATION_CREATE_MAPPED_BIT,
                  const std::vector<uint32_t>& queueIdxs = {});

  explicit Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&&) = delete;

  void     swap(Buffer& buffer);
  void     clear();
  uint8_t* map();
  void     unmap();
  void     flush();

  void update(std::vector<uint8_t>& data, size_t offset);
  void update(void* data, size_t size, size_t offset);
  void update(uint8_t* data, size_t size, size_t offset);

  const auto size() const { return mSize; }

public:
  vk::DeviceMemory mMemory;
  vk::DeviceSize   mSize;
  vk::DeviceSize   mCapacity;
  uint8_t*         mMapped_data;

protected:
  Device* mDevice;

  vk::BufferUsageFlags        mBufferUsage;
  vk::MemoryPropertyFlags     mRequired;
  vk::MemoryPropertyFlags     mPrefered;
  VmaAllocationCreateFlagBits mFlags;
  std::vector<uint32_t>       mQueueIdxs;
  VmaAllocation               mVmaAllocation;

  bool mMapped;
  bool mPersistent;
};
}  //namespace vkl