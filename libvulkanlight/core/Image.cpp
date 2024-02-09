#include "Image.h"
#include "Device.h"
#include "VklError.h"
#include "Logger.h"

namespace vkl {

Image::Image(Device* _device, vk::Image image, vk::ImageViewCreateInfo imageViewCI)
  : mDevice(_device)
  , mImageCI{}
  , mVmaAllocation{VK_NULL_HANDLE}
  , mTiling{}
  , mMapped{false}
  , mMappedData{nullptr} {
  mVkObject    = image;
  mVkFormat    = imageViewCI.format;
  mVkImageView = mDevice->vk().createImageView(imageViewCI);
}

Image::Image(Device*                 _device,
             vk::ImageCreateInfo     imageCI,
             vk::MemoryPropertyFlags required,
             vk::MemoryPropertyFlags prefered,
             vk::ImageViewCreateInfo imageViewCI)
  : mDevice(_device)
  , mImageCI{imageCI}
  , mVmaAllocation{VK_NULL_HANDLE}
  , mVkImageView{VK_NULL_HANDLE}
  , mMapped{false}
  , mMappedData{nullptr} {
  mVkFormat          = mImageCI.format;
  mTiling            = mImageCI.tiling;
  auto& vmaAllocator = mDevice->getMemoryAllocator();

  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.requiredFlags  = static_cast<VkMemoryPropertyFlags>(required);
  memoryInfo.preferredFlags = static_cast<VkMemoryPropertyFlags>(prefered);

  auto result = vmaCreateImage(vmaAllocator,
                               reinterpret_cast<VkImageCreateInfo const*>(&mImageCI),
                               &memoryInfo,
                               const_cast<VkImage*>(
                                 reinterpret_cast<VkImage const*>(&mVkObject)),
                               &mVmaAllocation,
                               nullptr);
  VKL_CHECK_ERROR(result, "create Image Fail");

  imageViewCI.image  = mVkObject;
  imageViewCI.format = mVkFormat;
  mVkImageView       = mDevice->vk().createImageView(imageViewCI);
}

Image::Image(Image&& other) noexcept
  : mDevice(nullptr)
  , mVmaAllocation(VK_NULL_HANDLE)
  , mVkImageView(VK_NULL_HANDLE)
  , mVkFormat(vk::Format::eUndefined) {
  swap(other);
}

Image::~Image() {
  if (mVkObject && mVmaAllocation) {
    vmaDestroyImage(mDevice->getMemoryAllocator(),
                    static_cast<VkImage>(mVkObject),
                    mVmaAllocation);
    mVkObject      = VK_NULL_HANDLE;
    mVmaAllocation = VK_NULL_HANDLE;
  }

  if (mVkImageView) {
    mDevice->vk().destroyImageView(mVkImageView);
    mVkImageView = VK_NULL_HANDLE;
  }
}

uint8_t* Image::map() {
  if (!mMappedData) {
    if (mTiling != vk::ImageTiling::eLinear) {
      vklLogW("Mapping image memory that is not linear");
    }

    auto result = vmaMapMemory(mDevice->getMemoryAllocator(),
                               mVmaAllocation,
                               reinterpret_cast<void**>(&mMappedData));
    VKL_CHECK_ERROR(result, "Image::map");

    mMapped = true;
  }
  return mMappedData;
}

void Image::unmap() {
  if (mMapped) {
    vmaUnmapMemory(mDevice->getMemoryAllocator(), mVmaAllocation);
    mMappedData = nullptr;
    mMapped     = false;
  }
}

void Image::swap(Image& image) {
  vk::Image     tempImg  = mVkObject;
  vk::ImageView tempView = mVkImageView;
  VmaAllocation tempAllo = mVmaAllocation;

  mDevice        = (image.mDevice);
  mVkObject      = (image.mVkObject);
  mVmaAllocation = (image.mVmaAllocation);
  mVkImageView   = (image.mVkImageView);
  mVkFormat      = (image.mVkFormat);

  image.mDevice        = mDevice;
  image.mVkObject      = tempImg;
  image.mVmaAllocation = tempAllo;
  image.mVkImageView   = tempView;
}

//void Image::unmap() {
//  if (mapped) {
//    vmaUnmapMemory(device->getVmaAllocator(), memory);
//    mapped_data = nullptr;
//    mapped      = false;
//  }
//}

}  //namespace vkl