#include <iostream>
#include <limits>
#include <memory>

#define VOLK_IMPLEMENTATION
#include <volk.h>

#include <vulkan/vulkan.hpp>

#include "App.h"
#include "Window.h"

#include "Device.h"
#include "Instance.h"
#include "Queue.h"
#include "ResourcePool.h"
#include "SwapchainRenderContext.h"
#include "UniformBuffer.h"

#include "GUI.h"
#include "GraphicsCamera.h"
#include "InputCallback.h"
#include "GraphicsPipeline.h"

#include "VklError.h"
#include "VklSettings.h"
#include "common.h"
#include "ShaderPool.h"

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE;

namespace vkl {

App::App()
  : mName{"App"}
  , mApiVersion{VK_API_VERSION_1_0}
  , mPrepared{false}
  , mEndApplication{true}
  , mInstance{nullptr}
  , mWindow{nullptr}
  , mVkSurface{VK_NULL_HANDLE}
  , mDevice{nullptr}
  , mRenderContext{VK_NULL_HANDLE}
  , mVkRenderPass{VK_NULL_HANDLE}
  , mVkFramebuffers{}
  , mVkDescPool{VK_NULL_HANDLE}
  , mCurrCmdFence{VK_NULL_HANDLE}
  , mCurrBufferingSemaphore({VK_NULL_HANDLE, VK_NULL_HANDLE})
  , mCurrBufferingIdx{0}
  , mLastSubpass{0}
  , mGUI{nullptr}  //, mGui{nullptr}
  , mInputCallback{nullptr}
  , mGraphicsCamera{nullptr} {
  VklLogger::init();
}

App::~App() {
  mDevice->vk().waitIdle();

  mCurrCmdFence           = VK_NULL_HANDLE;
  mCurrBufferingSemaphore = {VK_NULL_HANDLE, VK_NULL_HANDLE};

  if (mVkDescPool) {
    mDevice->vk().destroyDescriptorPool(mVkDescPool);
    mVkDescPool = VK_NULL_HANDLE;
  }

  mCameraDescriptor.cameraUB.reset();

  mGUI.reset();
  //mGui.reset();

  for (auto& frameBuffer : mVkFramebuffers) {
    mDevice->vk().destroyFramebuffer(frameBuffer);
  }
  mVkFramebuffers.clear();

  if (mVkRenderPass) {
    mDevice->vk().destroyRenderPass(mVkRenderPass);
  }

  mRenderContext.reset();

  ResourcePool::clear();

  mDevice.reset();

  if (mVkSurface) {
    mInstance->getVkInstance().destroySurfaceKHR(mVkSurface);
  }

  mInstance.reset();
}

void App::addShaderPath(const std::string& shaderPath) {
  ResourcePool::addShaderPath(shaderPath);
}

void App::addAssetPath(const std::string& assetPath) {
  ResourcePool::addAssetPath(assetPath);
}

void App::registerWindow(std::unique_ptr<Window>& _window) {
  mWindow = std::move(_window);
}

void App::onWindowResized(int w, int h, int orientation) {
  if (w * h == 0)
    return;

  if (!mRenderContext->getVkSwapchain()) {
    vklLogW("Can't handle surface changes in headless mode, skipping.");
    return;
  }

  vk::SurfaceCapabilitiesKHR surfaceProps = mDevice->getVkPhysicalDevice()
                                              .getSurfaceCapabilitiesKHR(mVkSurface);

  if (surfaceProps.currentExtent.width == 0xFFFFFFFF) {
    vklLogE("Surface Width is 0 ");
    return;
  }

  auto& prevExtent = mRenderContext->getContextProps().extent;
  if (surfaceProps.currentExtent.width != prevExtent.width
      || surfaceProps.currentExtent.height != prevExtent.height) {
    mDevice->vk().waitIdle();

    mWindow->updateExtent(surfaceProps.currentExtent.width,
                          surfaceProps.currentExtent.height);
    mRenderContext->resizeSwapChain();
    createFrameBuffer();

    mRenderContext->createCommandBuffer(vk::CommandBufferLevel::ePrimary);
    mDevice->vk().waitIdle();
  }

  mGUI->onWindowResized(w, h);

  if (mGraphicsCamera) {
    mGraphicsCamera->onWindowResized(w, h, orientation);

    auto count = mRenderContext->getContextImageCount();
    for (size_t i = 0; i < count; i++) {
      updateCameraUniform(i);
    }
  }
}

bool App::prepare() {
  static vk::DynamicLoader dl;
  VULKAN_HPP_DEFAULT_DISPATCHER.init(
    dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr"));

  //initialize volk
  VkResult result = volkInitialize();

  if (result) {
    VKL_CHECK_ERROR(result, "Failed to initialize volk.");
    return false;
  }

  createInstance();
  createDevice();

  ResourcePool::init(mDevice.get());

  /*render context parts*/
  createRenderContext();
  mRenderContext->prepare();

  createCommandPool();
  createCommandBuffer();

  //setup subpass
  createRenderPass();
  createFrameBuffer();

  createVkDescriptorPool();

  ShaderPool::init();

  createPipelines();
  createRenderers();

  createGUI();
  createGraphicsCamera();

  mPrepared = true;
  return mPrepared;
}

void App::run() {}

void App::end() {
  mEndApplication = true;
}

void App::onRender() {}

void App::createInstance() {
  VklSettings::availableInstanceExtProps = vk::enumerateInstanceExtensionProperties();

  auto& properties = VklSettings::availableInstanceExtProps;

  //get the surface extensions
  auto surfExtensions = mWindow->getRequiredSurfaceExtension();
  for (auto& surfExtension : surfExtensions) {
    VklSettings::addInstanceExtension(surfExtension, properties);
  }

#ifdef VKL_DEBUG

  VklSettings::hasDebugUtil =
    VklSettings::addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, properties);
  bool debugReport{false};
  if (!VklSettings::hasDebugUtil)
    debugReport = VklSettings::addInstanceExtension(VK_EXT_DEBUG_REPORT_EXTENSION_NAME,
                                                    properties);

  if (!VklSettings::hasDebugUtil && !debugReport) {
    vklLogW("Neither of {} or {} are available; disabling debug reporting",
            VK_EXT_DEBUG_UTILS_EXTENSION_NAME,
            VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
  }
#endif
  bool& headless = VklSettings::headless;
  headless       = mWindow->getWindowInfo().mode == WindowInfo::Mode::Headless;
  if (headless) {
    bool
      enable = VklSettings::addInstanceExtension(VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME,
                                                 properties);
    if (!enable) {
      VklSettings::addInstanceExtension(VK_KHR_SURFACE_EXTENSION_NAME, properties);
      headless = false;
    }
  }

#ifdef VKL_VALIDATION_LAYERS
  {
    VklSettings::availableLayers = vk::enumerateInstanceLayerProperties();
    vklLogD("availableLayers : {}", VklSettings::availableLayers.size());
    VklSettings::addLayer(VK_LAYER_KHRONOS_VALIDATION_NAME, VklSettings::availableLayers);
  }
#endif

  mInstance = std::make_unique<Instance>(mName,
                                         VklSettings::enabledInstanceExtensions,
                                         VklSettings::enabledLayers,
                                         mApiVersion);
  if (!mInstance) {
    vklLogE("failed to init mVkSurface");
    return;
  }
}

void App::createDevice() {
  mVkSurface     = mWindow->createSurface(mInstance.get());
  bool& headless = VklSettings::headless;

  if (!mVkSurface && !headless)
    throw std::runtime_error("Failed to create mWindow surface.");

  auto deviceCands = mInstance->getVkInstance().enumeratePhysicalDevices();
  if (deviceCands.empty()) {
    vklLogE("Could not find a physical mDevice");
    throw std::runtime_error("Could not find a physical mDevice");
  }

  vk::PhysicalDevice vkPhysicalDevice = deviceCands[0];
  for (auto& mDevice : deviceCands) {
    if (mDevice.getProperties().deviceType == vk::PhysicalDeviceType::eDiscreteGpu) {
      int queueCount = mDevice.getQueueFamilyProperties().size();
      for (uint32_t queue_idx = 0; static_cast<size_t>(queue_idx) < queueCount;
           queue_idx++) {
        if (mDevice.getSurfaceSupportKHR(queue_idx, mVkSurface)) {
          vkPhysicalDevice = mDevice;
          break;
        }
      }
    }
  }

  mDevice = std::make_unique<Device>(mInstance->getVkInstance(),
                                     vkPhysicalDevice,
                                     mVkSurface);

  VklSettings::availableDeviceExtensions = vkPhysicalDevice
                                             .enumerateDeviceExtensionProperties();

  auto& properties = VklSettings::availableDeviceExtensions;

  bool instanceEnabledHeadless = VklSettings::isInInstanceExtension(
    VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME);

  if (!headless || instanceEnabledHeadless) {
    VklSettings::addDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME, properties);

    if (VklSettings::isInInstanceExtension(VK_KHR_DISPLAY_EXTENSION_NAME)) {
      VklSettings::addDeviceExtension(VK_KHR_DISPLAY_SWAPCHAIN_EXTENSION_NAME,
                                      properties);
    }
  }

  bool canGetMemroy = VklSettings::isAvailableDeviceExtension(
    VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);

  bool hasDedicatedAlloc = VklSettings::isAvailableDeviceExtension(
    VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);

  if (canGetMemroy && hasDedicatedAlloc) {
    VklSettings::addDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
    VklSettings::addDeviceExtension(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);
  }

  //bool bufferDeivceAddr  = VklSettings::isAvailableDeviceExtension(
  //    VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  //bool deviceGroup =
  //VklSettings::isAvailableDeviceExtension(VK_KHR_DEVICE_GROUP_EXTENSION_NAME);

  //if (bufferDeivceAddr && deviceGroup) {
  //  VklSettings::addDeviceExtension(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  //  VklSettings::addDeviceExtension(VK_KHR_DEVICE_GROUP_EXTENSION_NAME);
  //}

#ifdef VKL_DEBUG
  if (!VklSettings::hasDebugUtil) {
    auto
      debugExtensionIt = std::find_if(properties.begin(),
                                      properties.end(),
                                      [](vk::ExtensionProperties const& ep) {
                                        return strcmp(ep.extensionName,
                                                      VK_EXT_DEBUG_MARKER_EXTENSION_NAME)
                                               == 0;
                                      });
    if (debugExtensionIt != properties.end()) {
      vklLogI("Vulkan debug utils enabled ({})", VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
      //YSTODO: debug marker
      //debug_utils =
      //std::make_unique<vkb::core::HPPDebugMarkerExtDebugUtils>();
      //add_device_extension(VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
    }
  }

#endif

  mDevice->initLogicalDevice();
}

void App::createRenderContext() {
  vk::PresentModeKHR present_mode = (mWindow->getWindowInfo().vsync
                                     == WindowInfo::Vsync::ON)
                                      ? vk::PresentModeKHR::eFifo
                                      : vk::PresentModeKHR::eMailbox;

  VklSettings::presentModePriorities = {vk::PresentModeKHR::eMailbox,
                                        vk::PresentModeKHR::eFifo,
                                        vk::PresentModeKHR::eImmediate};

  VklSettings::surfaceFormatPriorities = {
    {vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear},
    {vk::Format::eR8G8B8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear},
    { vk::Format::eR8G8B8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear},
    { vk::Format::eB8G8R8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear}
  };

  auto& presentableQueue = mDevice->getPresentableQueue();

  mRenderContext = std::make_unique<SwapchainRenderContext>(mDevice.get(),
                                                            mWindow.get(),
                                                            &presentableQueue,
                                                            present_mode);

  if (!mRenderContext)
    throw std::runtime_error("cannot create render context");
}

void App::createCommandPool() {
  mRenderContext->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
}

void App::createCommandBuffer() {
  mRenderContext->createCommandBuffer(vk::CommandBufferLevel::ePrimary);
}

void App::createRenderPass() {
  auto& scprops       = mRenderContext->getContextProps();
  auto  surfaceFormat = scprops.surfaceFormat;
  auto  depthFormat   = scprops.depthFormat;

  std::array<vk::AttachmentDescription, 2> attachments;
  //Color attachment
  attachments[0].format         = surfaceFormat.format;
  attachments[0].samples        = vk::SampleCountFlagBits::e1;
  attachments[0].loadOp         = vk::AttachmentLoadOp::eClear;
  attachments[0].storeOp        = vk::AttachmentStoreOp::eStore;
  attachments[0].stencilLoadOp  = vk::AttachmentLoadOp::eDontCare;
  attachments[0].stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
  attachments[0].initialLayout  = vk::ImageLayout::eUndefined;
  attachments[0].finalLayout    = vk::ImageLayout::ePresentSrcKHR;

  //Depth attachment
  attachments[1].format         = depthFormat;
  attachments[1].samples        = vk::SampleCountFlagBits::e1;
  attachments[1].loadOp         = vk::AttachmentLoadOp::eClear;
  attachments[1].storeOp        = vk::AttachmentStoreOp::eDontCare;
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
  //Subpass dependencies for layout transitions
  std::array<vk::SubpassDependency, 2> dependencies;

  dependencies[0].srcSubpass   = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass   = 0;
  dependencies[0].srcStageMask = vk::PipelineStageFlagBits::eBottomOfPipe;
  dependencies[0].dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput
                                 | vk::PipelineStageFlagBits::eEarlyFragmentTests
                                 | vk::PipelineStageFlagBits::eLateFragmentTests;
  dependencies[0].srcAccessMask = vk::AccessFlagBits::eNoneKHR;
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

void App::createFrameBuffer() {
  if (!mVkFramebuffers.empty()) {
    for (auto& framebuffer : mVkFramebuffers) {
      mDevice->vk().destroyFramebuffer(framebuffer);
    }
    mVkFramebuffers.clear();
  }

  auto swapSize = mRenderContext->getContextProps().imageCount;
  auto extent   = mRenderContext->getContextProps().extent;

  std::array<vk::ImageView, 2> attachments;
  attachments[1] = mRenderContext->getDepthStencilImage().front().vkImageView();

  vk::FramebufferCreateInfo framebufferCI({},
                                          mVkRenderPass,
                                          attachments,
                                          extent.width,
                                          extent.height,
                                          1);

  auto& swapchainImages = mRenderContext->getColorImages();

  mVkFramebuffers.reserve(swapSize);

  for (auto& image : swapchainImages) {
    attachments[0] = image.vkImageView();
    mVkFramebuffers.push_back(mDevice->vk().createFramebuffer(framebufferCI));
  }
}

void App::createVkDescriptorPool() {
  //Create Descriptor Pool
  std::vector<vk::DescriptorPoolSize> poolSizes = {
    {             vk::DescriptorType::eSampler, 30},
    {vk::DescriptorType::eCombinedImageSampler, 30},
    {        vk::DescriptorType::eSampledImage, 30},
    {        vk::DescriptorType::eStorageImage, 30},
    {       vk::DescriptorType::eUniformBuffer, 30},
    {  vk::DescriptorType::eStorageTexelBuffer, 30},
    {       vk::DescriptorType::eUniformBuffer, 30},
    {       vk::DescriptorType::eStorageBuffer, 30},
    {vk::DescriptorType::eUniformBufferDynamic, 30},
    {vk::DescriptorType::eStorageBufferDynamic, 30},
    {     vk::DescriptorType::eInputAttachment, 30}
  };

  vk::DescriptorPoolCreateInfo poolCI({}, 30 * 11, poolSizes);
  mVkDescPool = mDevice->vk().createDescriptorPool(poolCI);

  //VkDescriptorPoolCreateInfo pool_info = {};
  //pool_info.sType                      =
  //VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pool_info.flags         =
  //VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT; pool_info.maxSets = 30 *
  //11; pool_info.poolSizeCount = (uint32_t)11; pool_info.pPoolSizes    =
  //pool_sizes; err = vkCreateDescriptorPool(g_Device, &pool_info, g_Allocator,
  //&g_DescriptorPool); check_vk_result(err);
}

void App::createGUI() {
  ShaderSourceType type = ShaderSourceType::SPV;
  std::string      name = "imgui";

  auto* vert = ResourcePool::loadShader(name, type, vk::ShaderStageFlagBits::eVertex);
  auto* frag = ResourcePool::loadShader(name, type, vk::ShaderStageFlagBits::eFragment);

  auto* imguiPipelineLayout = ResourcePool::addPipelineLayout(vert, frag);

  constexpr char plname[] = "imgui";

  auto* guiPL = ResourcePool::addGraphicsPipeline(plname,
                                                  mRenderContext.get(),
                                                  mVkRenderPass,
                                                  imguiPipelineLayout,
                                                  0);

  mGUI = std::make_unique<GUI>();
  mGUI->prepare(mDevice.get(), mRenderContext.get(), mVkDescPool, mVkRenderPass, guiPL);

  mInputCallback   = std::make_unique<InputCallback>();
  auto keyCallback = [&](int key) {
    if (key == 526) {
      this->end();
    }
  };
  mInputCallback->registerKeyPressed(std::move(keyCallback));

  mGUI->addInputCallback(mInputCallback.get());
}

void App::createGraphicsCamera() {
  auto& extent    = mRenderContext->getContextProps().extent;
  mGraphicsCamera = std::make_unique<GraphicsCamera>(GraphicsCamera::VULKAN,
                                                     extent.width,
                                                     extent.height,
                                                     GraphicsCamera::PERSPECTIVE,
                                                     nullptr,
                                                     nullptr,
                                                     int(mWindow->getWindowInfo()
                                                           .orientation));

  mGUI->addInputCallback(mGraphicsCamera.get());

  auto count   = mRenderContext->getContextImageCount();
  auto memSize = sizeof(CameraUniform);

  vk::DescriptorSetLayoutBinding camBinding{0,
                                            vk::DescriptorType::eUniformBuffer,
                                            1,
                                            vk::ShaderStageFlagBits::eVertex};

  mCameraDescriptor.descLayout = ResourcePool::addDescriptorSetLayout("GraphicsCamera",
                                                                      {camBinding});
  mCameraDescriptor.descSets.resize(count);
  for (size_t i = 0u; i < count; ++i) {
    mCameraDescriptor.descSets[i] = mDevice->vk()
                                      .allocateDescriptorSets(
                                        {mVkDescPool, mCameraDescriptor.descLayout->vk()})
                                      .front();
  }

  //mCameraUB = UniformBuffer::Uni(
  //  new UniformBuffer(mDevice.get(), layout, mVkDescPool, 0, count,
  //  memSize));
  mCameraDescriptor.cameraUB = std::make_unique<UniformBuffer>(mDevice.get(),
                                                               mCameraDescriptor.descSets,
                                                               0,
                                                               memSize);
  CameraUniform tmp;
  for (int i = 0; i < count; ++i) {
    mCameraDescriptor.cameraUB->update(i, &tmp, 0);
  }
}

void App::createPipelines() {}

void App::createRenderers() {}

void App::requestGpuFeatures(Device* vkPhysicalDevice) {
  vklLogD("This Function should be overriden by app");
}

void App::buildCommandBuffer() {
  auto cmd = beginCommandBuffer();
  beginRenderPass(cmd);

  if (mGUI)
    mGUI->buildCommandBuffer(cmd, mCurrBufferingIdx);

  cmd.endRenderPass();
  cmd.end();
}

vk::CommandBuffer App::beginCommandBuffer() {
  //if you dont want to recycle
  vk::CommandBufferBeginInfo cmdBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  //if you want to recycle
  //vk::CommandBufferBeginInfo cmdBeginInfo;

  auto& renderCmdBuffers = mRenderContext->getRenderCommandBuffers();
  auto& swapChainImages  = mRenderContext->getColorImages();
  auto  cmdSize          = renderCmdBuffers.size();

  uint32_t queFamilyIdx = mRenderContext->getQueue()->getFamilyIdx();

  vk::CommandBuffer& renderCmdBuffer = renderCmdBuffers[mCurrBufferingIdx];
  vk::Image&         swapChainImage  = swapChainImages[mCurrBufferingIdx].vk();

  renderCmdBuffer.reset();
  renderCmdBuffer.begin(cmdBeginInfo);

  return renderCmdBuffer;
}

void App::beginRenderPass(vk::CommandBuffer renderCmdBuffer) {
  std::vector<vk::ClearValue> clearValues = {
    {vk::ClearColorValue(0.10f, 0.10f, 0.10f, 1.0f),
     vk::ClearDepthStencilValue(1.0f, 0)}
  };

  vk::RenderPassBeginInfo renderBeginInfo(mVkRenderPass,
                                          {},
                                          {{}, mRenderContext->getContextProps().extent},
                                          clearValues);

  renderBeginInfo.framebuffer = mVkFramebuffers[mCurrBufferingIdx];
  renderCmdBuffer.beginRenderPass(renderBeginInfo, vk::SubpassContents::eInline);

  auto extent = mRenderContext->getContextProps().extent;

  vk::Viewport viewport;
  viewport.width    = extent.width;
  viewport.height   = extent.height;
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;

  vk::Rect2D scissor({}, extent);

  renderCmdBuffer.setViewport(0, viewport);
  renderCmdBuffer.setScissor(0, scissor);
}

void App::updateCameraUniform(int idx) {
  auto& camUniformData = mGraphicsCamera->cam;
  auto  memSize        = sizeof(CameraUniform);
  mCameraDescriptor.cameraUB->update(idx, &camUniformData, memSize);
}

void App::updateUniform(int idx) {}

void App::prepareFrame() {
  //auto vkSwapchain = mRenderContext->getVkSwapchain();
  //if (!vkSwapchain)
  //  return;
  std::tie(mCurrBufferingIdx,
           mCurrCmdFence,
           mCurrBufferingSemaphore) = mRenderContext->acquireNextImage();

  //mCurrBufferingSemaphore           = mRenderContext->getCurrBufferingSemaphore();

  //auto& vkDevice = mDevice->vk();

  //auto result = vkDevice.acquireNextImageKHR(vkSwapchain,
  //                                           0xFFFFFFFFFFFFFFFF,
  //                                           mCurrBufferingSemaphore.available);
  //VKL_CHECK_ERROR(static_cast<VkResult>(result.result), "acquireNextImageKHR");

  //mCurrBufferingIdx = result.value;
  //mCurrCmdFence     = mRenderContext->getCurrCmdFence();

  //if (vkDevice.getFenceStatus(mCurrCmdFence) == vk::Result::eNotReady) {
  //  auto result = vkDevice.waitForFences({mCurrCmdFence}, VK_TRUE, UINT64_MAX);
  //}
  //vkDevice.resetFences({mCurrCmdFence});
}

void App::presentFrame() {
  auto& vkQueue   = mRenderContext->getVkQueue();
  auto& cmdBuffer = mRenderContext->getRenderCommandBuffers()[mCurrBufferingIdx];

  vk::PipelineStageFlags
    waitStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;

  vk::SubmitInfo submitInfo(mCurrBufferingSemaphore.available,
                            waitStageMask,
                            cmdBuffer,
                            mCurrBufferingSemaphore.rendered);

  vkQueue.submit(submitInfo, mCurrCmdFence);
  auto vkSwapchain = mRenderContext->getVkSwapchain();

  vk::PresentInfoKHR presentInfo(mCurrBufferingSemaphore.rendered,
                                 vkSwapchain,
                                 mCurrBufferingIdx);
  auto               result = vkQueue.presentKHR(presentInfo);
}

}  //namespace vkl