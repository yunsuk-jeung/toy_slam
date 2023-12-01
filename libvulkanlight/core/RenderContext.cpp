#include "RenderContext.h"
#include "Device.h"
#include "window/Window.h"
#include "VkSettings.h"
#include "VkLogger.h"

namespace vkl {
RenderContext::RenderContext(Device*            _device,
                             Window*            _window,
                             vk::PresentModeKHR presentMode)
  : device(_device)
  , window(_window) {
  ctProps.oldSwapchain   = nullptr;
  ctProps.imageCount     = 3;
  ctProps.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
  ctProps.extent.width   = window->getWindowInfo().extent.width;
  ctProps.extent.height  = window->getWindowInfo().extent.height;
  ctProps.presentMode    = presentMode;
}

RenderContext::~RenderContext() {
  vk::Device vkDevice = device->vk();
  vkDevice.waitIdle();

  if (renderCommandPool) { vkDevice.destroyCommandPool(renderCommandPool); }

  for (auto& fence : cmdFences) { vkDevice.destroyFence(fence); }

  for (auto& semaphore : bfSemaphores) {
    vkDevice.destroySemaphore(semaphore.available);
    vkDevice.destroySemaphore(semaphore.rendered);
  }

  device = nullptr;
  window = nullptr;
}

void RenderContext::prepare() {
  prepareColor();
  prepareDepthStencil();
}

void RenderContext::createCommandPool(vk::CommandPoolCreateFlagBits cmdPoolCreateFlag) {
  uint32_t famIdx = queue->getFamilyIdx();

  vk::CommandPoolCreateInfo poolInfo(cmdPoolCreateFlag, famIdx);
  renderCommandPool = device->vk().createCommandPool(poolInfo);
}

void RenderContext::createCommandBuffer(vk::CommandBufferLevel priority) {
  if (!renderCommandBuffers.empty()) {
    device->vk().freeCommandBuffers(renderCommandPool, renderCommandBuffers);
  }

  vk::CommandBufferAllocateInfo info(renderCommandPool, priority, ctProps.imageCount);

  renderCommandBuffers = device->vk().allocateCommandBuffers(info);
}

void RenderContext::resizeSwapChain() {}

vk::CommandBuffer RenderContext::getOneTimeCommandBuffer() {
  vk::CommandBufferAllocateInfo info(renderCommandPool,
                                     vk::CommandBufferLevel::ePrimary,
                                     1);

  auto cmdBuffers = device->vk().allocateCommandBuffers(info);
  return cmdBuffers.front();
}

void RenderContext::freeOneTimeCommandBuffer(vk::CommandBuffer cmd) {
  device->vk().freeCommandBuffers(renderCommandPool, cmd);
}

void RenderContext::prepareColor() {}

void RenderContext::prepareDepthStencil() {}

vk::Queue& RenderContext::getVkQueue() {
  return queue->getVkQueue();
}

vk::Fence RenderContext::getCurrCmdFence() {
  int curr = cmdFenceIdx++;
  if (cmdFenceIdx >= ctProps.imageCount) cmdFenceIdx = 0;
  return cmdFences[curr];
}

BufferingSemaphore RenderContext::getCurrBufferingSemaphore() {
  int curr = bfSemaphoreIdx++;
  if (bfSemaphoreIdx >= ctProps.imageCount) bfSemaphoreIdx = 0;
  return bfSemaphores[curr];
}

}  //namespace vkl