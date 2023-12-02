#pragma once
#include <memory>
#include <vector>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include "macros.h"

namespace vkl {
class Device;
class Buffer;
class BufferingBuffer {
public:
  USING_SMART_PTR(BufferingBuffer);
  DELETE_COPY_CONSTRUCTORS(BufferingBuffer);

  BufferingBuffer() = delete;

  explicit BufferingBuffer(
    uint32_t                     bufferCount,
    Device*                      device,
    vk::DeviceSize               size,
    vk::BufferUsageFlags         bufferUsage,
    vk::MemoryPropertyFlags      required,
    vk::MemoryPropertyFlags      prefered,
    VmaAllocationCreateFlagBits  flags     = VMA_ALLOCATION_CREATE_MAPPED_BIT,
    const std::vector<uint32_t>& queueIdxs = {});

protected:
  uint32_t                             mBufferCount;
  Device*                              mDevice;
  std::vector<std::unique_ptr<Buffer>> mBuffers;
  std::vector<size_t>                  mPrevSize;
};
}  //namespace vkl