#include <iostream>
#include "Buffer.h"
#include "Device.h"

#include "VkError.h"
#include "VkLogger.h"

namespace vkl {
//static int COUNTER = 0;
Buffer::~Buffer() {
  clear();
}

vkl::Buffer::Buffer(Device*                      _device,
                    vk::DeviceSize               _size,
                    vk::BufferUsageFlags         bufferUsage,
                    vk::MemoryPropertyFlags      required,
                    vk::MemoryPropertyFlags      prefered,
                    VmaAllocationCreateFlagBits  flags,
                    const std::vector<uint32_t>& queueIdxs)
  : device(_device)
  , size(_size)
  , memory{VK_NULL_HANDLE}
  , mapped_data{nullptr}
  , vmaAllocation{VK_NULL_HANDLE}
  , mapped{false}
  , persistent{false} {
  persistent = (flags & VMA_ALLOCATION_CREATE_MAPPED_BIT) != 0;

  vk::BufferCreateInfo buffer_create_info({}, size, bufferUsage);
  if (queueIdxs.size() >= 2) {
    buffer_create_info.sharingMode           = vk::SharingMode::eConcurrent;
    buffer_create_info.queueFamilyIndexCount = static_cast<uint32_t>(queueIdxs.size());
    buffer_create_info.pQueueFamilyIndices   = queueIdxs.data();
  }

  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.requiredFlags  = static_cast<VkMemoryPropertyFlags>(required);
  memoryInfo.preferredFlags = static_cast<VkMemoryPropertyFlags>(prefered);
  memoryInfo.flags          = flags;

  VmaAllocationInfo allocationInfo{};

  auto result = vmaCreateBuffer(device->getMemoryAllocator(),
                                reinterpret_cast<VkBufferCreateInfo*>(
                                  &buffer_create_info),
                                &memoryInfo,
                                reinterpret_cast<VkBuffer*>(&mVkObject),
                                &vmaAllocation,
                                &allocationInfo);

  if (result != VK_SUCCESS) {
    VK_CHECK_ERROR(result, "vma create buffer fail");
    throw std::runtime_error("failed to create buffer");
  }

  memory = static_cast<vk::DeviceMemory>(allocationInfo.deviceMemory);

  if (persistent) { mapped_data = static_cast<uint8_t*>(allocationInfo.pMappedData); }

  //std::cout << "create Buffer : " << ++COUNTER << std::endl;
}

Buffer::Buffer(Buffer&& src) noexcept {
  this->swap(src);
}

void vkl::Buffer::swap(Buffer& buffer) {
  auto _mVkObject     = mVkObject;
  auto _memory        = memory;
  auto _size          = size;
  auto _mapped_data   = mapped_data;
  auto _vmaAllocation = vmaAllocation;
  auto _device        = device;
  auto _mapped        = mapped;
  auto _persistent    = persistent;

  mVkObject     = buffer.mVkObject;
  memory        = buffer.memory;
  size          = buffer.size;
  mapped_data   = buffer.mapped_data;
  vmaAllocation = buffer.vmaAllocation;
  device        = buffer.device;
  mapped        = buffer.mapped;
  persistent    = buffer.persistent;

  buffer.mVkObject     = _mVkObject;
  buffer.memory        = _memory;
  buffer.size          = _size;
  buffer.mapped_data   = _mapped_data;
  buffer.vmaAllocation = _vmaAllocation;
  buffer.device        = _device;
  buffer.mapped        = _mapped;
  buffer.persistent    = _persistent;
}

void Buffer::clear() {
  if (mVkObject && (vmaAllocation != VK_NULL_HANDLE)) {
    unmap();
    vmaDestroyBuffer(device->getMemoryAllocator(),
                     static_cast<VkBuffer>(mVkObject),
                     vmaAllocation);
    mVkObject     = VK_NULL_HANDLE;
    vmaAllocation = VK_NULL_HANDLE;
  }
}

uint8_t* Buffer::map() {
  if (!mapped && !mapped_data) {
    VK_CHECK_ERROR(vmaMapMemory(device->getMemoryAllocator(),
                                vmaAllocation,
                                reinterpret_cast<void**>(&mapped_data)),
                   "");
    mapped = true;
  }
  return mapped_data;
}

void vkl::Buffer::unmap() {
  if (mapped) {
    vmaUnmapMemory(device->getMemoryAllocator(), vmaAllocation);
    mapped_data = nullptr;
    mapped      = false;
  }
}

void Buffer::flush() {
  vmaFlushAllocation(device->getMemoryAllocator(), vmaAllocation, 0, size);
}

void Buffer::update(std::vector<uint8_t>& data, size_t offset) {
  update(data.data(), data.size(), offset);
}

void Buffer::update(void* data, size_t size, size_t offset) {
  update((uint8_t*)data, size, offset);
}

void Buffer::update(uint8_t* data, size_t size, size_t offset) {
  if (persistent) {
    std::copy(data, data + size, mapped_data + offset);
    flush();
  }
  else {
    map();
    std::copy(data, data + size, mapped_data + offset);
    flush();
    unmap();
  }
}

}  //namespace vkl