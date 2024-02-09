#pragma once

#include <string>
#include <vulkan/vulkan.hpp>

namespace vkl {

namespace fs {
bool fileExists(const std::string& filename);
}

namespace cmd {
void setImageLayout(vk::CommandBuffer         cmdBuffer,
                    vk::Image                 vkImage,
                    vk::ImageLayout           oldLayout,
                    vk::ImageLayout           newLayout,
                    vk::ImageSubresourceRange subresourceRange,
                    vk::PipelineStageFlags    srcStageMask,
                    vk::PipelineStageFlags    dstStageMask);

void convertImageLayout(vk::CommandBuffer         cmdBuffer,
                        vk::Image                 vkImage,
                        vk::AccessFlagBits        srcAccessMask,
                        vk::AccessFlagBits        dstAccessMask,
                        vk::ImageLayout           oldLayout,
                        vk::ImageLayout           newLayout,
                        uint32_t                  srcQueueFamilyIndex_,
                        uint32_t                  dstQueueFamilyIndex_,
                        vk::ImageSubresourceRange subresourceRange,
                        vk::PipelineStageFlags    srcStageMask,
                        vk::PipelineStageFlags    dstStageMask);

}  //namespace cmd

namespace image {
vk::ImageCreateInfo createVkImageCI(
  vk::ImageType           type,
  vk::Format              format,
  vk::Extent3D            extent,
  vk::ImageUsageFlags     usage,
  uint32_t                mipLevels   = 1,
  uint32_t                arrayLayers = 1,
  vk::SampleCountFlagBits smaples     = vk::SampleCountFlagBits::e1,
  vk::ImageTiling         tilting     = vk::ImageTiling::eOptimal);

vk::ImageViewCreateInfo createVkImageViewCI(vk::ImageViewType    type,
                                            vk::ImageAspectFlags aspectMask,
                                            vk::Format           format,
                                            uint32_t             baseMipLevel   = 0,
                                            uint32_t             levelCount     = 1,
                                            uint32_t             baseArrayLayer = 0,
                                            uint32_t             layerCount     = 1);
};  //namespace image

}  //namespace vkl