#pragma once

#include <memory>
#include <vk_mem_alloc.h>
#include "Object.h"

namespace vkl {

class Device;
class Buffer : public Object<vk::Buffer> {
public:
  USING_SMART_PTR(Buffer);
  Buffer()              = default;
  Buffer(const Buffer&) = delete;
  ~Buffer();

  explicit Buffer(Device*                      device,
                  vk::DeviceSize               size,
                  vk::BufferUsageFlags         bufferUsage,
                  vk::MemoryPropertyFlags      required,
                  vk::MemoryPropertyFlags      prefered,
                  VmaAllocationCreateFlagBits  flags = VMA_ALLOCATION_CREATE_MAPPED_BIT,
                  const std::vector<uint32_t>& queueIdxs = {});

  explicit Buffer(Buffer&& other) noexcept;

  Buffer& operator=(const Buffer&) = delete;
  Buffer& operator=(Buffer&&)      = delete;

  void     swap(Buffer& buffer);
  void     clear();
  uint8_t* map();
  void     unmap();
  void     flush();

  void update(std::vector<uint8_t>& data, size_t offset);
  void update(void* data, size_t size, size_t offset);
  void update(uint8_t* data, size_t size, size_t offset);

public:
  vk::DeviceMemory memory;
  vk::DeviceSize   size;
  uint8_t*         mapped_data;

protected:
  VmaAllocation vmaAllocation;
  Device*       device;
  bool          mapped;
  bool          persistent;
};
}  //namespace vkl