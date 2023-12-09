#include "Buffer.h"
#include "BufferingBuffer.h"

namespace vkl {
BufferingBuffer::BufferingBuffer(Device*                      device,
                                 uint32_t                     bufferCount,
                                 vk::DeviceSize               size,
                                 vk::BufferUsageFlags         bufferUsage,
                                 vk::MemoryPropertyFlags      required,
                                 vk::MemoryPropertyFlags      prefered,
                                 VmaAllocationCreateFlagBits  flags,
                                 const std::vector<uint32_t>& queueIdxs)
  : mBufferCount{bufferCount}
  , mDevice{device} {
  mPrevSize.resize(mBufferCount, 0);
  mCapacities.resize(mBufferCount, size);
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

BufferingBuffer::~BufferingBuffer() {
  mBuffers.clear();
  mCapacities.clear();
  mPrevSize.clear();
}

void BufferingBuffer::update(uint32_t idx, std::vector<uint8_t>& data, size_t offset) {
  mBuffers[idx]->update(data, offset);
}

void BufferingBuffer::update(uint32_t idx, void* data, size_t size, size_t offset) {
  mBuffers[idx]->update(data, size, offset);
}

void BufferingBuffer::update(uint32_t idx, uint8_t* data, size_t size, size_t offset) {
  mBuffers[idx]->update(data, size, offset);
}

}  //namespace vkl