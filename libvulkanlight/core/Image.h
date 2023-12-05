#pragma once

#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include "macros.h"

namespace vkl {

class Device;
class Image {
public:
  USING_SMART_PTR(Image);
  Image() = default;

  //use to create image and allocate memroy at the same time
  //explicit Image(Device* _device, vk::ImageCreateInfo imageCI, VmaMemoryUsage memUsage);
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

public:
  vk::Image     vkImage       = VK_NULL_HANDLE;
  VmaAllocation vmaAllocation = VK_NULL_HANDLE;
  vk::ImageView vkImageView   = VK_NULL_HANDLE;
  vk::Format    format        = vk::Format::eUndefined;

protected:
  Device* device = nullptr;

  vk::ImageTiling tiling;
  bool            mapped      = false;
  uint8_t*        mapped_data = nullptr;
};

typedef std::unique_ptr<Image> ImagePtr;

}  //namespace vkl