#include "cores.h"
#include "utils.h"
#include "Window.h"
#include "SubApp.h"

namespace vkl {
SubApp::SubApp()
  : mWindow{nullptr}
  , mDevice{nullptr}
  , mRenderContext{nullptr}
  , mVkRenderPass{VK_NULL_HANDLE}
  , mVkFramebuffers{}
  , mVkDescPool{nullptr}
  , mCurrCmdFence{VK_NULL_HANDLE}
  , mCurrBufferingSemaphore{VK_NULL_HANDLE, VK_NULL_HANDLE}
  , mCurrBufferingIdx{0}
  , mPrepared{false}
  , mEndApplication{false}

{}

SubApp::~SubApp() {
  if (mVkRenderPass) {
    mDevice->vk().destroyRenderPass(mVkRenderPass);
  }
  for (auto& frameBuffer : mVkFramebuffers) {
    mDevice->vk().destroyFramebuffer(frameBuffer);
  }
  mVkFramebuffers.clear();
  mRenderContext.reset();

  mDevice     = nullptr;
  mVkDescPool = VK_NULL_HANDLE;
}

void SubApp::registerWindow(std::unique_ptr<Window>& window) {
  mWindow = std::move(window);
}

bool SubApp::prepare(Device* device, vk::DescriptorPool descPool) {
  mDevice     = device;
  mVkDescPool = descPool;

  createRenderContext();
  createCommandPool();
  createCommandBuffer();
  mRenderContext->prepare();

  createRenderPass();
  createFrameBuffer();

  createPipelines();
  createRenderers();

  mPrepared = true;
  return mPrepared;
}

void SubApp::run() {
  mThread = std::thread([&]() { this->onRender(); });
}

void SubApp::end() {
  mEndApplication = true;
  if (mThread.joinable())
    mThread.join();
}

void SubApp::createRenderContext() {
  auto* queue = &mDevice->getLowPrioritydQueue(vk::QueueFlagBits::eGraphics);

  mRenderContext = std::make_unique<RenderContext>(mDevice, mWindow.get(), queue);
}

void SubApp::createCommandPool() {
  mRenderContext->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
}

void SubApp::createCommandBuffer() {
  mRenderContext->createCommandBuffer(vk::CommandBufferLevel::ePrimary);
}

void SubApp::createRenderPass() {
  auto& scprops     = mRenderContext->getContextProps();
  auto  colorFormat = scprops.surfaceFormat.format;
  auto  depthFormat = scprops.depthFormat;

  std::array<vk::AttachmentDescription, 2> attachments;
  //Color attachment
  attachments[0].format         = colorFormat;
  attachments[0].samples        = vk::SampleCountFlagBits::e1;
  attachments[0].loadOp         = vk::AttachmentLoadOp::eClear;
  attachments[0].storeOp        = vk::AttachmentStoreOp::eStore;
  attachments[0].stencilLoadOp  = vk::AttachmentLoadOp::eDontCare;
  attachments[0].stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
  attachments[0].initialLayout  = vk::ImageLayout::eUndefined;
  attachments[0].finalLayout    = vk::ImageLayout::eColorAttachmentOptimal;

  //Depth attachment
  attachments[1].format         = depthFormat;
  attachments[1].samples        = vk::SampleCountFlagBits::e1;
  attachments[1].loadOp         = vk::AttachmentLoadOp::eClear;
  attachments[1].storeOp        = vk::AttachmentStoreOp::eStore;
  attachments[1].stencilLoadOp  = vk::AttachmentLoadOp::eClear;
  attachments[1].stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
  attachments[1].initialLayout  = vk::ImageLayout::eUndefined;
  attachments[1].finalLayout    = vk::ImageLayout::eDepthStencilAttachmentOptimal;

  vk::AttachmentReference colorRef(0, vk::ImageLayout::eColorAttachmentOptimal);
  vk::AttachmentReference depthRef(1, vk::ImageLayout::eDepthStencilAttachmentOptimal);

  vk::SubpassDescription subpassDescription({},
                                            vk::PipelineBindPoint::eGraphics,
                                            {},
                                            colorRef,
                                            {},
                                            &depthRef);

  std::array<vk::SubpassDependency, 2> dependencies;

  dependencies[0].srcSubpass   = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass   = 0;
  dependencies[0].srcStageMask = vk::PipelineStageFlagBits::eBottomOfPipe;
  dependencies[0].dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput
                                 | vk::PipelineStageFlagBits::eEarlyFragmentTests
                                 | vk::PipelineStageFlagBits::eLateFragmentTests;
  dependencies[0].srcAccessMask = vk::AccessFlagBits::eNone;
  dependencies[0].dstAccessMask = vk::AccessFlagBits::eColorAttachmentRead
                                  | vk::AccessFlagBits::eColorAttachmentWrite
                                  | vk::AccessFlagBits::eDepthStencilAttachmentRead
                                  | vk::AccessFlagBits::eDepthStencilAttachmentWrite;
  dependencies[0].dependencyFlags = vk::DependencyFlagBits::eByRegion;

  dependencies[1].srcSubpass   = 0;
  dependencies[1].dstSubpass   = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput
                                 | vk::PipelineStageFlagBits::eEarlyFragmentTests
                                 | vk::PipelineStageFlagBits::eLateFragmentTests;
  dependencies[1].dstStageMask  = vk::PipelineStageFlagBits::eBottomOfPipe;
  dependencies[1].srcAccessMask = vk::AccessFlagBits::eColorAttachmentRead
                                  | vk::AccessFlagBits::eColorAttachmentWrite
                                  | vk::AccessFlagBits::eDepthStencilAttachmentRead
                                  | vk::AccessFlagBits::eDepthStencilAttachmentWrite;
  dependencies[1].dstAccessMask   = vk::AccessFlagBits::eMemoryRead;
  dependencies[1].dependencyFlags = vk::DependencyFlagBits::eByRegion;

  vk::RenderPassCreateInfo renderPassCI({},
                                        attachments,
                                        subpassDescription,
                                        dependencies);

  mVkRenderPass = mDevice->vk().createRenderPass(renderPassCI);
}

void SubApp::createFrameBuffer() {
  if (!mVkFramebuffers.empty()) {
    for (auto& framebuffer : mVkFramebuffers) {
      mDevice->vk().destroyFramebuffer(framebuffer);
    }
    mVkFramebuffers.clear();
  }

  auto swapSize = mRenderContext->getContextProps().imageCount;
  auto extent   = mRenderContext->getContextProps().extent;

  std::array<vk::ImageView, 2> attachments;

  vk::FramebufferCreateInfo framebufferCI({},
                                          mVkRenderPass,
                                          attachments,
                                          extent.width,
                                          extent.height,
                                          1);

  auto& colors = mRenderContext->getColorImages();
  auto& depths = mRenderContext->getDepthStencilImage();

  mVkFramebuffers.reserve(swapSize);

  for (uint32_t i = 0u; i < swapSize; ++i) {
    attachments[0] = colors[i].vkImageView();
    attachments[1] = depths[i].vkImageView();
    mVkFramebuffers.push_back(mDevice->vk().createFramebuffer(framebufferCI));
  }
}

void SubApp::createPipelines() {}

void SubApp::createRenderers() {}

void SubApp::buildCommandBuffer() {}

void SubApp::updateUniforms(uint32_t idx) {}

void SubApp::onRender() {}

void SubApp::prepareFrame() {
  std::tie(mCurrBufferingIdx,
           mCurrCmdFence,
           mCurrBufferingSemaphore) = mRenderContext->acquireNextImage();

  /*mCurrBufferingIdx                 = mRenderContext->acquireNextImage();
  mCurrCmdFence                     = mRenderContext->getCurrCmdFence();
  mCurrBufferingSemaphore           = mRenderContext->getCurrBufferingSemaphore();*/

  auto vkDevice = mDevice->vk();

  if (vkDevice.getFenceStatus(mCurrCmdFence) == vk::Result::eNotReady) {
    auto result = vkDevice.waitForFences({mCurrCmdFence}, VK_TRUE, UINT64_MAX);
    VKL_CHECK_ERROR2(result, "waitfence");
  }
  vkDevice.resetFences({mCurrCmdFence});
}

void SubApp::submitCommandBuffer() {
  auto& vkQueue   = mRenderContext->getVkQueue();
  auto& cmdBuffer = mRenderContext->getRenderCommandBuffers()[mCurrBufferingIdx];

  vk::PipelineStageFlags
    waitStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;

  vk::SubmitInfo submitInfo;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers    = &cmdBuffer;
  submitInfo.pWaitDstStageMask  = &waitStageMask;
  //submitInfo.signalSemaphoreCount = 1;
  //submitInfo.pSignalSemaphores  = &currBfSemaphore.rendered;

  vkQueue.submit(submitInfo, mCurrCmdFence);

  auto vkDevice = mDevice->vk();
  auto result   = vkDevice.waitForFences({mCurrCmdFence}, VK_TRUE, UINT64_MAX);
  VKL_CHECK_ERROR2(result, "waitfence");
}
}  //namespace vkl