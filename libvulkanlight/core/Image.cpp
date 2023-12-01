#include "Image.h"
#include "Device.h"
#include "VkError.h"
#include "VkLogger.h"

namespace vkl {

Image::Image(Device* _device, vk::Image image, vk::ImageViewCreateInfo imageViewCI)
  : device(_device)
  , vkImage(image) {
  format      = imageViewCI.format;
  vkImageView = device->vk().createImageView(imageViewCI);
}

Image::Image(Device*                 _device,
             vk::ImageCreateInfo     imageCI,
             vk::MemoryPropertyFlags required,
             vk::MemoryPropertyFlags prefered,
             vk::ImageViewCreateInfo imageViewCI)
  : device(_device) {
  format             = imageCI.format;
  tiling             = imageCI.tiling;
  auto& vmaAllocator = device->getMemoryAllocator();

  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.requiredFlags  = static_cast<VkMemoryPropertyFlags>(required);
  memoryInfo.preferredFlags = static_cast<VkMemoryPropertyFlags>(prefered);

  auto result = vmaCreateImage(vmaAllocator,
                               reinterpret_cast<VkImageCreateInfo const*>(&imageCI),
                               &memoryInfo,
                               const_cast<VkImage*>(
                                 reinterpret_cast<VkImage const*>(&vkImage)),
                               &vmaAllocation,
                               nullptr);
  VK_CHECK_ERROR(result, "create Image Fail");

  imageViewCI.image  = vkImage;
  imageViewCI.format = format;
  vkImageView        = device->vk().createImageView(imageViewCI);
}

Image::Image(Image&& other) noexcept
  : device(nullptr)
  , vkImage(VK_NULL_HANDLE)
  , vmaAllocation(VK_NULL_HANDLE)
  , vkImageView(VK_NULL_HANDLE)
  , format(vk::Format::eUndefined) {
  swap(other);
}

Image::~Image() {
  if (vkImage && vmaAllocation) {
    vmaDestroyImage(device->getMemoryAllocator(),
                    static_cast<VkImage>(vkImage),
                    vmaAllocation);
    vkImage       = VK_NULL_HANDLE;
    vmaAllocation = VK_NULL_HANDLE;
  }

  if (vkImageView) {
    device->vk().destroyImageView(vkImageView);
    vkImageView = VK_NULL_HANDLE;
  }
}

uint8_t* Image::map() {
  if (!mapped_data) {
    if (tiling != vk::ImageTiling::eLinear) {
      VklLogW("Mapping image memory that is not linear");
    }

    auto result = vmaMapMemory(device->getMemoryAllocator(),
                               vmaAllocation,
                               reinterpret_cast<void**>(&mapped_data));
    VK_CHECK_ERROR(result, "Image::map");

    mapped = true;
  }
  return mapped_data;
}

void Image::unmap() {
  if (mapped) {
    vmaUnmapMemory(device->getMemoryAllocator(), vmaAllocation);
    mapped_data = nullptr;
    mapped      = false;
  }
}

void Image::swap(Image& image) {
  vk::Image     tempImg  = vkImage;
  vk::ImageView tempView = vkImageView;
  VmaAllocation tempAllo = vmaAllocation;

  device        = (image.device);
  vkImage       = (image.vkImage);
  vmaAllocation = (image.vmaAllocation);
  vkImageView   = (image.vkImageView);
  format        = (image.format);

  image.device        = device;
  image.vkImage       = tempImg;
  image.vmaAllocation = tempAllo;
  image.vkImageView   = tempView;
}

//void Image::unmap() {
//  if (mapped) {
//    vmaUnmapMemory(device->getVmaAllocator(), memory);
//    mapped_data = nullptr;
//    mapped      = false;
//  }
//}

}  //namespace vkl