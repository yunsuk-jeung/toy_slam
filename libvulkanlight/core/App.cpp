#include <iostream>
#include <memory>
#include <limits>

#define VOLK_IMPLEMENTATION
#include <volk.h>

#include <vulkan/vulkan.hpp>

#include "App.h"
#include "Window.h"

#include "Instance.h"
#include "Device.h"
#include "SwapchainRenderContext.h"
#include "Queue.h"
#include "UniformBuffer.h"
#include "ResourcePool.h"

#include "GUI.h"
#include "InputCallback.h"
#include "GraphicsCamera.h"
#include "Pipeline.h"

#include "VkError.h"
#include "VkLogger.h"
#include "VkSettings.h"
#include "shaders.h"
#include "Utils.h"
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
  spdlog::set_level(spdlog::level::debug);
}

App::~App() {
  mDevice->vk().waitIdle();

  mCurrCmdFence           = VK_NULL_HANDLE;
  mCurrBufferingSemaphore = {VK_NULL_HANDLE, VK_NULL_HANDLE};

  if (mVkDescPool) {
    mDevice->vk().destroyDescriptorPool(mVkDescPool);
    mVkDescPool = VK_NULL_HANDLE;
  }

  mCameraUB.reset();

  mGUI.reset();
  //mGui.reset();

  for (auto& frameBuffer : mVkFramebuffers) {
    mDevice->vk().destroyFramebuffer(frameBuffer);
  }
  mVkFramebuffers.clear();

  if (mVkRenderPass) { mDevice->vk().destroyRenderPass(mVkRenderPass); }

  mRenderContext.reset();

  ResourcePool::clear(mDevice.get());

  mDevice.reset();

  if (mVkSurface) { mInstance->getVkInstance().destroySurfaceKHR(mVkSurface); }

  mInstance.reset();
}

void App::setShaderPath(const std::string& shaderPath) {
  ResourcePool::setShaderPath(shaderPath);
}

void App::setResourcePath(const std::string& resourcePath) {
  ResourcePool::setResourcePath(resourcePath);
}

void App::registerWindow(std::unique_ptr<Window>& _window) {
  mWindow = std::move(_window);
}

void App::onWindowResized(int w, int h, int orientation) {
  if (!mRenderContext->getVkSwapchain()) {
    VklLogW("Can't handle surface changes in headless mode, skipping.");
    return;
  }

  vk::SurfaceCapabilitiesKHR surfaceProps = mDevice->getVkPhysicalDevice()
                                              .getSurfaceCapabilitiesKHR(mVkSurface);

  if (surfaceProps.currentExtent.width == 0xFFFFFFFF) {
    VklLogE("Surface Width is 0 ");
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
    //buildCommandBuffers();
    mDevice->vk().waitIdle();
  }

  mGUI->onWindowResized(w, h);

  if (mGraphicsCamera) {
    mGraphicsCamera->onWindowResized(w, h, orientation);

    auto count = mRenderContext->getContextImageCount();
    for (size_t i = 0; i < count; i++) { updateCameraUniform(i); }
  }
}

bool App::prepare() {
  static vk::DynamicLoader dl;
  VULKAN_HPP_DEFAULT_DISPATCHER.init(
    dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr"));

  //initialize volk
  VkResult result = volkInitialize();

  if (result) {
    VK_CHECK_ERROR(result, "Failed to initialize volk.");
    return false;
  }

  createInstance();
  createDevice();

  /*render context parts*/
  createRenderContext();
  mRenderContext->prepare();

  createCommandPool();
  createCommandBuffer();

  //setup subpass
  createRenderPass();
  createFrameBuffer();

  createPipelineLayouts();
  createPipelines();

  createVkDescriptorPool();

  createGUI();
  createGraphicsCamera();

  mPrepared = true;
  return mPrepared;
}

void App::run() {}

void App::end() {}

void App::onRender() {}

void App::createInstance() {
  VkSettings::availableInstanceExtProps = vk::enumerateInstanceExtensionProperties();

  auto& properties = VkSettings::availableInstanceExtProps;

  //get the surface extensions
  auto surfExtensions = mWindow->getRequiredSurfaceExtension();
  for (auto& surfExtension : surfExtensions) {
    VkSettings::addInstanceExtension(surfExtension, properties);
  }

#ifdef VKL_DEBUG

  VkSettings::hasDebugUtil =
    VkSettings::addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, properties);
  bool debugReport{false};
  if (!VkSettings::hasDebugUtil)
    debugReport = VkSettings::addInstanceExtension(VK_EXT_DEBUG_REPORT_EXTENSION_NAME,
                                                   properties);

  if (!VkSettings::hasDebugUtil && !debugReport) {
    VklLogW("Neither of {} or {} are available; disabling debug reporting",
            VK_EXT_DEBUG_UTILS_EXTENSION_NAME,
            VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
  }
#endif
  bool& headless = VkSettings::headless;
  headless       = mWindow->getWindowInfo().mode == WindowInfo::Mode::Headless;
  if (headless) {
    bool enable = VkSettings::addInstanceExtension(VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME,
                                                   properties);
    if (!enable) {
      VkSettings::addInstanceExtension(VK_KHR_SURFACE_EXTENSION_NAME, properties);
      headless = false;
    }
  }

#ifdef VKL_VALIDATION_LAYERS
  {
    VkSettings::availableLayers = vk::enumerateInstanceLayerProperties();
    VklLogD("availableLayers : {}", VkSettings::availableLayers.size());
    VkSettings::addLayer(VK_LAYER_KHRONOS_VALIDATION_NAME, VkSettings::availableLayers);
  }
#endif

  mInstance = std::make_unique<Instance>(mName,
                                         VkSettings::enabledInstanceExtensions,
                                         VkSettings::enabledLayers,
                                         mApiVersion);
  if (!mInstance) {
    VklLogE("failed to init mVkSurface");
    return;
  }
}

void App::createDevice() {
  mVkSurface     = mWindow->createSurface(mInstance.get());
  bool& headless = VkSettings::headless;

  if (!mVkSurface && !headless)
    throw std::runtime_error("Failed to create mWindow surface.");

  auto deviceCands = mInstance->getVkInstance().enumeratePhysicalDevices();
  if (deviceCands.empty()) {
    VklLogE("Could not find a physical mDevice");
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

  VkSettings::availableDeviceExtensions = vkPhysicalDevice
                                            .enumerateDeviceExtensionProperties();

  auto& properties = VkSettings::availableDeviceExtensions;

  bool instanceEnabledHeadless = VkSettings::isInInstanceExtension(
    VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME);

  if (!headless || instanceEnabledHeadless) {
    VkSettings::addDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME, properties);

    if (VkSettings::isInInstanceExtension(VK_KHR_DISPLAY_EXTENSION_NAME)) {
      VkSettings::addDeviceExtension(VK_KHR_DISPLAY_SWAPCHAIN_EXTENSION_NAME, properties);
    }
  }

  bool canGetMemroy = VkSettings::isAvailableDeviceExtension(
    VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);

  bool hasDedicatedAlloc = VkSettings::isAvailableDeviceExtension(
    VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);

  if (canGetMemroy && hasDedicatedAlloc) {
    VkSettings::addDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
    VkSettings::addDeviceExtension(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);
  }

  //bool bufferDeivceAddr  = VkSettings::isAvailableDeviceExtension(
  //    VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  //bool deviceGroup =
  //VkSettings::isAvailableDeviceExtension(VK_KHR_DEVICE_GROUP_EXTENSION_NAME);

  //if (bufferDeivceAddr && deviceGroup) {
  //  VkSettings::addDeviceExtension(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  //  VkSettings::addDeviceExtension(VK_KHR_DEVICE_GROUP_EXTENSION_NAME);
  //}

#ifdef VKL_DEBUG
  if (!VkSettings::hasDebugUtil) {
    auto
      debugExtensionIt = std::find_if(properties.begin(),
                                      properties.end(),
                                      [](vk::ExtensionProperties const& ep) {
                                        return strcmp(ep.extensionName,
                                                      VK_EXT_DEBUG_MARKER_EXTENSION_NAME)
                                               == 0;
                                      });
    if (debugExtensionIt != properties.end()) {
      VklLogI("Vulkan debug utils enabled ({})", VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
      //TODO: debug marker
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

  VkSettings::presentModePriorities = {vk::PresentModeKHR::eMailbox,
                                       vk::PresentModeKHR::eFifo,
                                       vk::PresentModeKHR::eImmediate};

  VkSettings::surfaceFormatPriorities = {
    {vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear},
    {vk::Format::eR8G8B8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear},
    { vk::Format::eR8G8B8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear},
    { vk::Format::eB8G8R8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear}
  };

  mRenderContext = std::make_unique<SwapchainRenderContext>(mDevice.get(),
                                                            mWindow.get(),
                                                            present_mode);

  if (!mRenderContext) throw std::runtime_error("cannot create render context");
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
  attachments[1] = mRenderContext->getDepthStencilImage().front().vkImageView;

  vk::FramebufferCreateInfo framebufferCI({},
                                          mVkRenderPass,
                                          attachments,
                                          extent.width,
                                          extent.height,
                                          1);

  auto& swapchainImages = mRenderContext->getColorImages();

  mVkFramebuffers.reserve(swapSize);

  for (auto& image : swapchainImages) {
    attachments[0] = image.vkImageView;
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
  //VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT; pool_info.maxSets       = 30 *
  //11; pool_info.poolSizeCount = (uint32_t)11; pool_info.pPoolSizes    = pool_sizes;
  //err = vkCreateDescriptorPool(g_Device, &pool_info, g_Allocator, &g_DescriptorPool);
  //check_vk_result(err);
}

void App::createPipelineLayouts() {
  ShaderSourceType  type           = ShaderSourceType::SPV;
  std::string       name           = "imgui";
  const std::string vertexShader   = std::string((const char*)shader::imguiVertspv,
                                               sizeof(shader::imguiVertspv));
  const std::string fragmentShader = std::string((const char*)shader::imguiFragspv,
                                                 sizeof(shader::imguiFragspv));

  auto* vert = ResourcePool::loadShader(name,
                                        mDevice.get(),
                                        type,
                                        vk::ShaderStageFlagBits::eVertex,
                                        vertexShader);

  auto* frag = ResourcePool::loadShader(name,
                                        mDevice.get(),
                                        type,
                                        vk::ShaderStageFlagBits::eFragment,
                                        fragmentShader);

  ResourcePool::addPipelineLayout(mDevice.get(), vert, frag);
}

void App::createPipelines() {
  auto* imguiPipelineLayout = ResourcePool::requestPipelineLayout(
    "imgui_vert_imgui_frag");

  auto* imguiPipeline = new ImGuiPipeline("imgui",
                                          mDevice.get(),
                                          mRenderContext.get(),
                                          mVkRenderPass,
                                          imguiPipelineLayout,
                                          0);
  imguiPipeline->prepare();
}

void App::createGUI() {
  mGUI = GUI::Uni(new GUI());
  mGUI->prepare(mDevice.get(), mRenderContext.get(), mVkDescPool, mVkRenderPass, "imgui");

  mInputCallback   = InputCallback::Uni(new InputCallback());
  auto keyCallback = [&](int key) {
    if (key == 526) { this->mEndApplication = true; }
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

  //mGraphicsCamera->init(extent.width, extent.height);
  mGUI->addInputCallback(mGraphicsCamera.get());

  auto count   = mRenderContext->getContextImageCount();
  auto memSize = sizeof(CameraUniform);
  mCameraUB    = UniformBuffer::Uni(new UniformBuffer(mDevice.get(), count, memSize));

  CameraUniform tmp;
  for (int i = 0; i < count; ++i) { mCameraUB->update(i, &tmp, 0); }

  vk::DescriptorSetLayoutBinding camBinding{0,
                                            vk::DescriptorType::eUniformBuffer,
                                            1,
                                            vk::ShaderStageFlagBits::eVertex};

  mCameraUB->createDescSets(camBinding, mVkDescPool);

  std::vector<vk::DescriptorPoolSize> poolSizes = {
    {vk::DescriptorType::eUniformBuffer, 3}
  };
}

void App::requestGpuFeatures(Device* vkPhysicalDevice) {
  VklLogD("This Function should be overriden by app");
}

void App::buildCommandBuffer() {}

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
  vk::Image&         swapChainImage  = swapChainImages[mCurrBufferingIdx].vkImage;

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
  mCameraUB->update(idx, &camUniformData, memSize);
}

void App::updateUniform(int idx) {}

void App::prepareFrame() {
  auto vkSwapchain = mRenderContext->getVkSwapchain();
  if (!vkSwapchain) return;

  mCurrBufferingSemaphore = mRenderContext->getCurrBufferingSemaphore();

  auto vkDevice = mDevice->vk();

  auto result = vkDevice.acquireNextImageKHR(vkSwapchain,
                                             0xFFFFFFFFFFFFFFFF,
                                             mCurrBufferingSemaphore.available);
  VK_CHECK_ERROR(static_cast<VkResult>(result.result), "acquireNextImageKHR");

  mCurrBufferingIdx = result.value;
  mCurrCmdFence     = mRenderContext->getCurrCmdFence();

  if (vkDevice.getFenceStatus(mCurrCmdFence) == vk::Result::eNotReady) {
    auto result = vkDevice.waitForFences({mCurrCmdFence}, VK_TRUE, UINT64_MAX);
  }
  vkDevice.resetFences({mCurrCmdFence});
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