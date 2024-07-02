#pragma once
#include <set>
#include <vector>
#include <vulkan/vulkan.hpp>
#include "RenderContext.h"

namespace vkl {

class Device;
class Queue;
class Window;
class SwapchainRenderContext : public RenderContext {
public:
  SwapchainRenderContext() = delete;
  SwapchainRenderContext(Device*            _device,
                         Window*            _window,
                         Queue*             _queue,
                         vk::PresentModeKHR presentMode);
  ~SwapchainRenderContext();

  void prepare() override;
  void resizeSwapChain() override;

  virtual std::tuple<uint32_t, vk::Fence, BufferingSemaphore> acquireNextImage() override;

protected:
  void prepareColor() override;
  void prepareDepthStencil() override;

  void createSwapChain(vk::Extent2D                    extent     = {},
                       uint32_t                        imageCount = 4,
                       vk::SurfaceTransformFlagBitsKHR transform =
                         vk::SurfaceTransformFlagBitsKHR::eIdentity,
                       std::set<vk::ImageUsageFlagBits> imageUsageFlags =
                         {vk::ImageUsageFlagBits::eColorAttachment,
                          vk::ImageUsageFlagBits::eTransferSrc},
                       vk::SwapchainKHR oldSwapchain = nullptr);

  void setSwapChainProperties(vk::Extent2D                     extent,
                              uint32_t                         imageCount,
                              vk::SurfaceTransformFlagBitsKHR  transform,
                              std::set<vk::ImageUsageFlagBits> imageUsageFlags,
                              vk::SwapchainKHR                 oldSwapchain);

protected:

public:
};
}  //namespace vkl