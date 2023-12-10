#include <iostream>
#include "Buffer.h"
#include "Device.h"

#include "VkError.h"
#include "VklLogger.h"

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
  : mDevice(_device)
  , mSize(_size)
  , mCapacity(_size << 1)
  , mBufferUsage{bufferUsage}
  , mRequired{required}
  , mPrefered{prefered}
  , mFlags{flags}
  , mQueueIdxs{queueIdxs}
  , mMemory{VK_NULL_HANDLE}
  , mMapped_data{nullptr}
  , mVmaAllocation{VK_NULL_HANDLE}
  , mMapped{false}
  , mPersistent{false} {
  mPersistent = (mFlags & VMA_ALLOCATION_CREATE_MAPPED_BIT) != 0;

  vk::BufferCreateInfo buffer_create_info({}, mCapacity, mBufferUsage);
  if (mQueueIdxs.size() >= 2) {
    buffer_create_info.sharingMode           = vk::SharingMode::eConcurrent;
    buffer_create_info.queueFamilyIndexCount = static_cast<uint32_t>(mQueueIdxs.size());
    buffer_create_info.pQueueFamilyIndices   = mQueueIdxs.data();
  }

  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.requiredFlags  = static_cast<VkMemoryPropertyFlags>(mRequired);
  memoryInfo.preferredFlags = static_cast<VkMemoryPropertyFlags>(mPrefered);
  memoryInfo.flags          = mFlags;

  VmaAllocationInfo allocationInfo{};

  auto result = vmaCreateBuffer(mDevice->getMemoryAllocator(),
                                reinterpret_cast<VkBufferCreateInfo*>(
                                  &buffer_create_info),
                                &memoryInfo,
                                reinterpret_cast<VkBuffer*>(&mVkObject),
                                &mVmaAllocation,
                                &allocationInfo);

  if (result != VK_SUCCESS) {
    VK_CHECK_ERROR(result, "vma create buffer fail");
    throw std::runtime_error("failed to create buffer");
  }

  mMemory = static_cast<vk::DeviceMemory>(allocationInfo.deviceMemory);

  if (mPersistent) {
    mMapped_data = static_cast<uint8_t*>(allocationInfo.pMappedData);
  }

  //std::cout << "create Buffer : " << ++COUNTER << std::endl;
}

Buffer::Buffer(Buffer&& src) noexcept {
  this->swap(src);
}

void vkl::Buffer::swap(Buffer& buffer) {
  auto vkObject      = mVkObject;
  auto memory        = mMemory;
  auto size          = mSize;
  auto capacity      = mCapacity;
  auto mapped_data   = mMapped_data;
  auto device        = mDevice;
  auto bufferUsage   = mBufferUsage;
  auto required      = mRequired;
  auto prefered      = mPrefered;
  auto flags         = mFlags;
  auto queueIdxs     = mQueueIdxs;
  auto vmaAllocation = mVmaAllocation;
  auto mapped        = mMapped;
  auto persistent    = mPersistent;

  mVkObject      = buffer.mVkObject;
  mMemory        = buffer.mMemory;
  mSize          = buffer.mSize;
  mCapacity      = buffer.mCapacity;
  mMapped_data   = buffer.mMapped_data;
  mDevice        = buffer.mDevice;
  mBufferUsage   = buffer.mBufferUsage;
  mRequired      = buffer.mRequired;
  mPrefered      = buffer.mPrefered;
  mFlags         = buffer.mFlags;
  mQueueIdxs     = buffer.mQueueIdxs;
  mVmaAllocation = buffer.mVmaAllocation;
  mMapped        = buffer.mMapped;
  mPersistent    = buffer.mPersistent;

  buffer.mVkObject      = vkObject;
  buffer.mMemory        = memory;
  buffer.mSize          = size;
  buffer.mCapacity      = capacity;
  buffer.mMapped_data   = mapped_data;
  buffer.mDevice        = device;
  buffer.mBufferUsage   = bufferUsage;
  buffer.mRequired      = required;
  buffer.mPrefered      = prefered;
  buffer.mFlags         = flags;
  buffer.mQueueIdxs     = queueIdxs;
  buffer.mVmaAllocation = vmaAllocation;
  buffer.mMapped        = mapped;
  buffer.mPersistent    = persistent;
}

void Buffer::clear() {
  if (mVkObject && (mVmaAllocation != VK_NULL_HANDLE)) {
    unmap();
    vmaDestroyBuffer(mDevice->getMemoryAllocator(),
                     static_cast<VkBuffer>(mVkObject),
                     mVmaAllocation);
    mVkObject      = VK_NULL_HANDLE;
    mVmaAllocation = VK_NULL_HANDLE;
  }
}

uint8_t* Buffer::map() {
  if (!mMapped && !mMapped_data) {
    VK_CHECK_ERROR(vmaMapMemory(mDevice->getMemoryAllocator(),
                                mVmaAllocation,
                                reinterpret_cast<void**>(&mMapped_data)),
                   "");
    mMapped = true;
  }
  return mMapped_data;
}

void vkl::Buffer::unmap() {
  if (mMapped) {
    vmaUnmapMemory(mDevice->getMemoryAllocator(), mVmaAllocation);
    mMapped_data = nullptr;
    mMapped      = false;
  }
}

void Buffer::flush() {
  vmaFlushAllocation(mDevice->getMemoryAllocator(), mVmaAllocation, 0, mSize);
}

void Buffer::update(std::vector<uint8_t>& data, size_t offset) {
  update(data.data(), data.size(), offset);
}

void Buffer::update(void* data, size_t size, size_t offset) {
  update((uint8_t*)data, size, offset);
}

void Buffer::update(uint8_t* data, size_t size, size_t offset) {
  if (offset + size > mCapacity) {
    auto newSize = offset + size;

    Buffer buffer = Buffer(mDevice,
                           newSize,
                           mBufferUsage,
                           mRequired,
                           mPrefered,
                           mFlags,
                           mQueueIdxs);
    this->swap(buffer);
  }

  if (mPersistent) {
    std::copy(data, data + size, mMapped_data + offset);
    flush();
  }
  else {
    map();
    std::copy(data, data + size, mMapped_data + offset);
    flush();
    unmap();
  }
}

}  //namespace vkl