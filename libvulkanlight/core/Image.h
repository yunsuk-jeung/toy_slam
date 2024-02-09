#pragma once

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.hpp>
#include "macros.h"
#include "VkObject.h"

namespace vkl {

class Device;
class Image : public VkObject<vk::Image> {
public:
  USING_SMART_PTR(Image);
  Image() = default;

  //use to create image and allocate memroy at the same time
  //explicit Image(Device* _device, vk::ImageCreateInfo imageCI, VmaMemoryUsage
  //memUsage);
  explicit Image(Device*                 _device,
                 vk::ImageCreateInfo     imageCI,
                 vk::MemoryPropertyFlags required,
                 vk::MemoryPropertyFlags prefered,
                 vk::ImageViewCreateInfo imageViewCI);

  //use when vk image already exist (swap chain)
  explicit Image(Device* _device, vk::Image image, vk::ImageViewCreateInfo imageViewCI);

  ~Image();

  DELETE_COPY_CONSTRUCTORS(Image);

  Image(Image&& other) noexcept;
  Image& operator=(Image&&) = delete;

  uint8_t* map();
  void     unmap();

protected:
  void swap(Image& image);
  //void unmap();

protected:
  Device*             mDevice;
  vk::ImageCreateInfo mImageCI;
  vk::ImageView       mVkImageView;
  vk::Format          mVkFormat;
  VmaAllocation       mVmaAllocation;
  vk::ImageTiling     mTiling;
  bool                mMapped;
  uint8_t*            mMappedData = nullptr;

public:
  const vk::ImageView& vkImageView() const { return mVkImageView; }
  const vk::Format&    vkFormat() const { return mVkFormat; }
  const vk::Extent3D&  extent() const { return mImageCI.extent; }
};

typedef std::unique_ptr<Image> ImagePtr;

}  //namespace vkl