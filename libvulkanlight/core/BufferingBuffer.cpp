#include "Buffer.h"
#include "BufferingBuffer.h"

namespace vkl {
BufferingBuffer::BufferingBuffer(uint32_t                     bufferCount,
                                 Device*                      device,
                                 vk::DeviceSize               size,
                                 vk::BufferUsageFlags         bufferUsage,
                                 vk::MemoryPropertyFlags      required,
                                 vk::MemoryPropertyFlags      prefered,
                                 VmaAllocationCreateFlagBits  flags,
                                 const std::vector<uint32_t>& queueIdxs)
  : mBufferCount{bufferCount}
  , mDevice{device} {
  mPrevSize.resize(mBufferCount, size);
  mBuffers.resize(mBufferCount);
  for (auto& buffer : mBuffers) {
    buffer = std::make_unique<Buffer>(mDevice,
                                      size,
                                      bufferUsage,
                                      required,
                                      prefered,
                                      flags,
                                      queueIdxs);
  }
}
}  //namespace vkl