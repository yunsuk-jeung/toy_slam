#include "RenderContext.h"
#include "Device.h"
#include "Logger.h"
#include "VklSettings.h"
#include "common.h"
#include "Window.h"

namespace vkl {
RenderContext::RenderContext(Device* _device, Window* _window, Queue* queue)
  : mDevice(_device)
  , mWindow(_window)
  , mQueue{queue}
  , mCurrentBufferingIdx{0} {
  mCtProps.oldSwapchain   = nullptr;
  mCtProps.imageCount     = 3;
  mCtProps.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
  mCtProps.extent.width   = mWindow->getWindowInfo().extent.width;
  mCtProps.extent.height  = mWindow->getWindowInfo().extent.height;
  mCtProps.surfaceFormat  = {vk::Format::eB8G8R8A8Unorm,
                             vk::ColorSpaceKHR::eSrgbNonlinear};

  mVkImageUsageFlags.emplace(vk::ImageUsageFlagBits::eSampled);
  mVkImageUsageFlags.emplace(vk::ImageUsageFlagBits::eColorAttachment);
  mVkImageUsageFlags.emplace(vk::ImageUsageFlagBits::eTransferSrc);
}

RenderContext::~RenderContext() {
  vk::Device vkDevice = mDevice->vk();
  vkDevice.waitIdle();

  if (mRenderCommandPool) {
    vkDevice.destroyCommandPool(mRenderCommandPool);
  }

  for (auto& fence : mCmdFences) {
    vkDevice.destroyFence(fence);
  }

  for (auto& semaphore : mBfSemaphores) {
    vkDevice.destroySemaphore(semaphore.available);
    vkDevice.destroySemaphore(semaphore.rendered);
  }

  mDevice = nullptr;
  mWindow = nullptr;
}

void RenderContext::prepare() {
  prepareColor();
  prepareDepthStencil();
}

void RenderContext::createCommandPool(vk::CommandPoolCreateFlagBits cmdPoolCreateFlag) {
  uint32_t famIdx = mQueue->getFamilyIdx();

  vk::CommandPoolCreateInfo poolInfo(cmdPoolCreateFlag, famIdx);
  mRenderCommandPool = mDevice->vk().createCommandPool(poolInfo);
}

void RenderContext::createCommandBuffer(vk::CommandBufferLevel priority) {
  if (!mRenderCommandBuffers.empty()) {
    mDevice->vk().freeCommandBuffers(mRenderCommandPool, mRenderCommandBuffers);
  }

  vk::CommandBufferAllocateInfo info(mRenderCommandPool, priority, mCtProps.imageCount);

  mRenderCommandBuffers = mDevice->vk().allocateCommandBuffers(info);
}

void RenderContext::resizeSwapChain() {}

vk::CommandBuffer RenderContext::getOneTimeCommandBuffer(vk::CommandBufferLevel level,
                                                         bool                   begin) {
  vk::CommandBufferAllocateInfo info(mRenderCommandPool, level, 1);

  auto cmdBuffers = mDevice->vk().allocateCommandBuffers(info);
  auto cmd        = cmdBuffers.front();
  if (begin) {
    vk::CommandBufferBeginInfo beginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
    cmd.begin(beginInfo);
  }

  return cmd;
}

void RenderContext::freeOneTimeCommandBuffer(vk::CommandBuffer cmd) {
  mDevice->vk().freeCommandBuffers(mRenderCommandPool, cmd);
}

void RenderContext::prepareColor() {
  auto vkDevice = mDevice->vk();

  auto imageSize = mCtProps.imageCount;
  mColorImages.reserve(imageSize);

  uint32_t w, h;
  w           = mCtProps.extent.width;
  h           = mCtProps.extent.height;
  auto format = mCtProps.surfaceFormat.format;

  vk::ImageUsageFlags colorUsageFlags;
  for (auto& flag : mVkImageUsageFlags) {
    colorUsageFlags |= flag;
  }

  vk::ImageCreateInfo imageCI = image::createVkImageCI(vk::ImageType::e2D,
                                                       format,
                                                       {w, h, 1},
                                                       colorUsageFlags);

  vk::ImageViewCreateInfo
    imageViewCI = image::createVkImageViewCI(vk::ImageViewType::e2D,
                                             vk::ImageAspectFlagBits::eColor,
                                             format);
  for (int i = 0; i < imageSize; ++i) {
    mColorImages.emplace_back(mDevice,
                              imageCI,
                              vk::MemoryPropertyFlagBits::eDeviceLocal,
                              vk::MemoryPropertyFlagBits::eDeviceLocal,
                              imageViewCI);
  }

  if (mCmdFences.empty()) {
    vk::FenceCreateInfo fenceInfo(vk::FenceCreateFlagBits::eSignaled);
    mCmdFences.reserve(imageSize);
    mBfSemaphores.reserve(imageSize);

    for (int i = 0; i < imageSize; ++i) {
      mCmdFences.emplace_back(vkDevice.createFence(fenceInfo));
      mBfSemaphores.push_back(
        {vkDevice.createSemaphore({}), vkDevice.createSemaphore({})});
    }
  }
}

void RenderContext::prepareDepthStencil() {
  mCtProps.depthFormat = mDevice->getSuitableDepthFormat();
  auto vkDevice        = mDevice->vk();

  auto imageSize = mCtProps.imageCount;
  mDepthStencilImages.reserve(imageSize);

  uint32_t w, h;
  w           = mCtProps.extent.width;
  h           = mCtProps.extent.height;
  auto format = mCtProps.depthFormat;

  vk::ImageCreateInfo
    imageCI = image::createVkImageCI(vk::ImageType::e2D,
                                     format,
                                     {w, h, 1},
                                     vk::ImageUsageFlagBits::eSampled
                                       | vk::ImageUsageFlagBits::eDepthStencilAttachment);

  vk::ImageViewCreateInfo
    imageViewCI = image::createVkImageViewCI(vk::ImageViewType::e2D,
                                             vk::ImageAspectFlagBits::eDepth,
                                             format);

  if (vk::Format::eD16UnormS8Uint <= mCtProps.depthFormat) {
    imageViewCI.subresourceRange.aspectMask |= vk::ImageAspectFlagBits::eStencil;
  }

  for (int i = 0; i < imageSize; ++i) {
    mDepthStencilImages.emplace_back(mDevice,
                                     imageCI,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal,
                                     imageViewCI);
  }
}

std::tuple<uint32_t, vk::Fence, BufferingSemaphore> RenderContext::acquireNextImage() {
  uint32_t curr = mCurrentBufferingIdx++;
  if (mCurrentBufferingIdx >= mCtProps.imageCount)
    mCurrentBufferingIdx = 0;
  return std::make_tuple(curr, mCmdFences[curr], mBfSemaphores[curr]);
}

vk::Queue& RenderContext::getVkQueue() {
  return mQueue->vk();
}

//vk::Fence RenderContext::getCurrCmdFence() {
//  int curr = cmdFenceIdx++;
//  if (cmdFenceIdx >= ctProps.imageCount)
//    cmdFenceIdx = 0;
//  return cmdFences[curr];
//}
//
//BufferingSemaphore RenderContext::getCurrBufferingSemaphore() {
//  int curr = bfSemaphoreIdx++;
//  if (bfSemaphoreIdx >= ctProps.imageCount)
//    bfSemaphoreIdx = 0;
//  return bfSemaphores[curr];
//}

}  //namespace vkl