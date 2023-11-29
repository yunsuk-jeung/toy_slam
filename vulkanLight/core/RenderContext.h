#pragma once
#include <vector>
#include <set>
#include <vulkan/vulkan.hpp>

#include "core/Image.h"
#include "core/types.h"

namespace vkl {

class Device;
class Queue;
class Window;
class RenderContext {
public:
  RenderContext() = delete;
  RenderContext(Device* _device, Window* _window, vk::PresentModeKHR presentMode);
  virtual ~RenderContext();

  virtual void prepare();
  void         createCommandPool(vk::CommandPoolCreateFlagBits cmdPoolCreateFlag);
  void         createCommandBuffer(vk::CommandBufferLevel priority);
  virtual void resizeSwapChain();

  vk::CommandBuffer getOneTimeCommandBuffer();
  void              freeOneTimeCommandBuffer(vk::CommandBuffer cmd);

protected:
  virtual void prepareColor();
  virtual void prepareDepthStencil();

protected:
  Window* window{nullptr};

  Device* device{nullptr};
  Queue*  queue{nullptr};

  ContextProps                     ctProps;
  std::set<vk::ImageUsageFlagBits> vkImageUsageFlags;
  vk::SwapchainKHR                 vkSwapchain{VK_NULL_HANDLE};

  uint32_t                        cmdFenceIdx{0};
  std::vector<vk::Fence>          cmdFences;
  uint32_t                        bfSemaphoreIdx{0};
  std::vector<BufferingSemaphore> bfSemaphores;

  std::vector<Image> colorImages;
  std::vector<Image> depthStencilImages;

  vk::CommandPool                renderCommandPool;
  std::vector<vk::CommandBuffer> renderCommandBuffers;

public:
  Window*          getWindow() { return window; }
  Queue*           getQueue() { return queue; }
  vk::SwapchainKHR getVkSwapchain() { return vkSwapchain; }

  ContextProps&       getContextProps() { return ctProps; }
  std::vector<Image>& getColorImages() { return colorImages; };
  uint32_t            getContextImageCount() { return ctProps.imageCount; }
  std::vector<Image>& getDepthStencilImage() { return depthStencilImages; }

  vk::Queue& getVkQueue();

  vk::CommandPool&                getRenderCommandPool() { return renderCommandPool; }
  std::vector<vk::CommandBuffer>& getRenderCommandBuffers() {
    return renderCommandBuffers;
  }
  vk::Fence          getCurrCmdFence();
  BufferingSemaphore getCurrBufferingSemaphore();
};
}  //namespace vkl