#pragma once
#include <vulkan/vulkan.hpp>

#include "Image.h"

namespace vkl {

struct ContextProps {
  vk::SwapchainKHR                oldSwapchain;
  uint32_t                        imageCount;
  vk::Extent2D                    extent;
  vk::SurfaceFormatKHR            surfaceFormat;
  uint32_t                        arrayLayers;
  vk::ImageUsageFlags             imageUsage;
  vk::SurfaceTransformFlagBitsKHR preTransform;
  vk::CompositeAlphaFlagBitsKHR   compositeAlpha;
  vk::PresentModeKHR              presentMode;
  vk::Format                      depthFormat;
};

struct BufferingSemaphore {
  vk::Semaphore available{VK_NULL_HANDLE};
  vk::Semaphore rendered{VK_NULL_HANDLE};
};

enum class DistortionModel {
  RADTAN,
  EQUI,
};

}  //namespace vkl