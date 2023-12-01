#include "Utils.h"

namespace vkl {
namespace cmd {
void setImageLayout(vk::CommandBuffer         cmdBuffer,
                    vk::Image                 vkImage,
                    vk::ImageLayout           oldLayout,
                    vk::ImageLayout           newLayout,
                    vk::ImageSubresourceRange subresourceRange,
                    vk::PipelineStageFlags    srcStageMask,
                    vk::PipelineStageFlags    dstStageMask) {
  vk::ImageMemoryBarrier barrier({},
                                 {},
                                 oldLayout,
                                 newLayout,
                                 VK_QUEUE_FAMILY_IGNORED,
                                 VK_QUEUE_FAMILY_IGNORED,
                                 vkImage,
                                 subresourceRange);
  switch (oldLayout) {
  case vk::ImageLayout::eUndefined:
    //Image layout is undefined (or does not matter)
    //Only valid as initial layout
    //No flags required, listed only for completeness
    barrier.srcAccessMask = vk::AccessFlagBits::eNone;
    break;

  case vk::ImageLayout::ePreinitialized:
    //Image is preinitialized
    //Only valid as initial layout for linear images, preserves memory contents
    //Make sure host writes have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eHostWrite;
    break;

  case vk::ImageLayout::eColorAttachmentOptimal:
    //Image is a color attachment
    //Make sure any writes to the color buffer have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;
    break;

  case vk::ImageLayout::eDepthStencilAttachmentOptimal:
    //Image is a depth/stencil attachment
    //Make sure any writes to the depth/stencil buffer have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    break;

  case vk::ImageLayout::eTransferSrcOptimal:
    //Image is a transfer source
    //Make sure any reads from the image have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferRead;
    break;

  case vk::ImageLayout::eTransferDstOptimal:
    //Image is a transfer destination
    //Make sure any writes to the image have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    break;

  case vk::ImageLayout::eShaderReadOnlyOptimal:
    //Image is read by a shader
    //Make sure any shader reads from the image have been finished
    barrier.srcAccessMask = vk::AccessFlagBits::eShaderRead;
    break;
  default:
    //Other source layouts aren't handled (yet)
    break;
  }

  //Target layouts (new)
  //Destination access mask controls the dependency for the new image layout
  switch (newLayout) {
  case vk::ImageLayout::eTransferDstOptimal:
    //Image will be used as a transfer destination
    //Make sure any writes to the image have been finished
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;
    break;

  case vk::ImageLayout::eTransferSrcOptimal:
    //Image will be used as a transfer source
    //Make sure any reads from the image have been finished
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferRead;
    break;

  case vk::ImageLayout::eColorAttachmentOptimal:
    //Image will be used as a color attachment
    //Make sure any writes to the color buffer have been finished
    barrier.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;
    break;

  case vk::ImageLayout::eDepthStencilAttachmentOptimal:
    //Image layout will be used as a depth/stencil attachment
    //Make sure any writes to depth/stencil buffer have been finished
    barrier.dstAccessMask = barrier.dstAccessMask
                            | vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    break;

  case vk::ImageLayout::eShaderReadOnlyOptimal:
    //Image will be read in a shader (sampler, input attachment)
    //Make sure any writes to the image have been finished
    //if (barrier.srcAccessMask == vk::AccessFlagBits::eNone) {
    //  barrier.srcAccessMask
    //      = vk::AccessFlagBits::eHostWrite | vk::AccessFlagBits::eTransferWrite;
    //}
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
    break;
  default:
    //Other source layouts aren't handled (yet)
    break;
  }

  cmdBuffer.pipelineBarrier(srcStageMask, dstStageMask, {}, {}, {}, barrier);
}
void convertImageLayout(vk::CommandBuffer         cmdBuffer,
                        vk::Image                 vkImage,
                        vk::AccessFlagBits        srcAccessMask,
                        vk::AccessFlagBits        dstAccessMask,
                        vk::ImageLayout           oldLayout,
                        vk::ImageLayout           newLayout,
                        uint32_t                  srcQueueFamilyIndex,
                        uint32_t                  dstQueueFamilyIndex,
                        vk::ImageSubresourceRange subresourceRange,
                        vk::PipelineStageFlags    srcStageMask,
                        vk::PipelineStageFlags    dstStageMask) {
  vk::ImageMemoryBarrier srcBarrier(srcAccessMask,
                                    dstAccessMask,
                                    oldLayout,
                                    newLayout,
                                    srcQueueFamilyIndex,
                                    dstQueueFamilyIndex,
                                    vkImage,
                                    subresourceRange);

  cmdBuffer.pipelineBarrier(srcStageMask, dstStageMask, {}, {}, {}, srcBarrier);
}
}  //namespace cmd
std::string Utils::shaderPath;
std::string Utils::resourcePath;

vk::ImageCreateInfo Utils::createVkImageCI(vk::ImageType           type,
                                           vk::Format              format,
                                           vk::Extent3D            extent,
                                           vk::ImageUsageFlags     usage,
                                           uint32_t                mipLevels,
                                           uint32_t                arrayLayers,
                                           vk::SampleCountFlagBits smaples,
                                           vk::ImageTiling         tilting) {
  vk::ImageCreateInfo imageCI;
  imageCI.imageType = type;
  imageCI.format    = format;
  imageCI.extent    = extent;
  imageCI.usage     = usage;

  imageCI.mipLevels   = mipLevels;
  imageCI.arrayLayers = arrayLayers;
  imageCI.samples     = smaples;
  imageCI.tiling      = tilting;

  return imageCI;
}

vk::ImageViewCreateInfo Utils::createVkImageViewCI(vk::ImageViewType    type,
                                                   vk::ImageAspectFlags aspectMask,
                                                   uint32_t             baseMipLevel,
                                                   uint32_t             levelCount,
                                                   uint32_t             baseArrayLayer,
                                                   uint32_t             layerCount) {
  vk::ImageViewCreateInfo imageViewCI;
  imageViewCI.viewType                        = type;
  imageViewCI.subresourceRange.aspectMask     = aspectMask;
  imageViewCI.subresourceRange.baseMipLevel   = baseMipLevel;
  imageViewCI.subresourceRange.levelCount     = levelCount;
  imageViewCI.subresourceRange.baseArrayLayer = baseArrayLayer;
  imageViewCI.subresourceRange.layerCount     = layerCount;

  return imageViewCI;
}

void Utils::setShaderPath(std::string path) {
  shaderPath = path;
}

std::string& Utils::getShaderPath() {
  return shaderPath;
}

void Utils::setResourcePath(std::string path) {
  resourcePath = path;
}

std::string& Utils::getResourcePath() {
  return resourcePath;
}
}  //namespace vkl