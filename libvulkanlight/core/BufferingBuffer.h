#pragma once
#include <memory>
#include <vector>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include "macros.h"
#include "Buffer.h"

namespace vkl {
class Device;
class Buffer;
class BufferingBuffer {
public:
  USING_SMART_PTR(BufferingBuffer);
  DELETE_COPY_CONSTRUCTORS(BufferingBuffer);

  BufferingBuffer() = delete;
  explicit BufferingBuffer(
    Device*                      device,
    uint32_t                     bufferCount,
    vk::DeviceSize               size,
    vk::BufferUsageFlags         bufferUsage,
    vk::MemoryPropertyFlags      required,
    vk::MemoryPropertyFlags      prefered,
    VmaAllocationCreateFlagBits  flags     = VMA_ALLOCATION_CREATE_MAPPED_BIT,
    const std::vector<uint32_t>& queueIdxs = {});

  ~BufferingBuffer();

  Buffer* at(uint32_t idx) { return mBuffers[idx].get(); }
  Buffer* operator[](uint32_t idx) { return mBuffers[idx].get(); }

  uint8_t* map(uint32_t idx) { return mBuffers[idx]->map(); };
  void     unmap(uint32_t idx) { return mBuffers[idx]->unmap(); };
  void     flush(uint32_t idx) { return mBuffers[idx]->flush(); };

  void update(uint32_t idx, std::vector<uint8_t>& data, size_t offset);
  void update(uint32_t idx, void* data, size_t size, size_t offset);
  void update(uint32_t idx, uint8_t* data, size_t size, size_t offset);

protected:
  uint32_t                             mBufferCount;
  Device*                              mDevice;
  std::vector<std::unique_ptr<Buffer>> mBuffers;
  std::vector<size_t>                  mPrevSize;
  std::vector<size_t>                  mCapacities;
};
}  //namespace vkl