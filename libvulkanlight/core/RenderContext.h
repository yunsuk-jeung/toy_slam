#pragma once
#include <set>
#include <vector>
#include <tuple>
#include <vulkan/vulkan.hpp>

#include "Image.h"
#include "vkltypes.h"

namespace vkl {

class Device;
class Queue;
class Window;
class RenderContext {
public:
  RenderContext() = delete;
  RenderContext(Device* _device, Window* _window, Queue* queue);
  virtual ~RenderContext();

  virtual void prepare();
  void         createCommandPool(vk::CommandPoolCreateFlagBits cmdPoolCreateFlag);
  void         createCommandBuffer(vk::CommandBufferLevel priority);
  virtual void resizeSwapChain();

  vk::CommandBuffer getOneTimeCommandBuffer(vk::CommandBufferLevel level,
                                            bool                   begin = false);
  void              freeOneTimeCommandBuffer(vk::CommandBuffer cmd);

  virtual std::tuple<uint32_t, vk::Fence, BufferingSemaphore> acquireNextImage();

protected:
  virtual void prepareColor();
  virtual void prepareDepthStencil();

protected:
  Window* mWindow{nullptr};

  Device* mDevice{nullptr};
  Queue*  mQueue{nullptr};

  ContextProps                     mCtProps;
  std::set<vk::ImageUsageFlagBits> mVkImageUsageFlags;
  vk::SwapchainKHR                 mVkSwapchain{VK_NULL_HANDLE};

  //uint32_t cmdFenceIdx{0};
  //uint32_t bfSemaphoreIdx{0};
  uint32_t                        mCurrentBufferingIdx;
  std::vector<vk::Fence>          mCmdFences;
  std::vector<BufferingSemaphore> mBfSemaphores;

  std::vector<Image> mColorImages;
  std::vector<Image> mDepthStencilImages;

  vk::CommandPool                mRenderCommandPool;
  std::vector<vk::CommandBuffer> mRenderCommandBuffers;

public:
  Window*          getWindow() { return mWindow; }
  Queue*           getQueue() { return mQueue; }
  vk::SwapchainKHR getVkSwapchain() { return mVkSwapchain; }

  ContextProps&       getContextProps() { return mCtProps; }
  std::vector<Image>& getColorImages() { return mColorImages; };
  uint32_t            getContextImageCount() { return mCtProps.imageCount; }
  std::vector<Image>& getDepthStencilImage() { return mDepthStencilImages; }

  vk::Queue& getVkQueue();

  vk::CommandPool&                getRenderCommandPool() { return mRenderCommandPool; }
  std::vector<vk::CommandBuffer>& getRenderCommandBuffers() {
    return mRenderCommandBuffers;
  }

  //vk::Fence getCurrCmdFence();
  //BufferingSemaphore getCurrBufferingSemaphore();
};
}  //namespace vkl