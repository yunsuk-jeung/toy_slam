#pragma once

#include <iostream>
#include <memory>
#include <limits>

#define VOLK_IMPLEMENTATION
#include <Volk/volk.h>

#include <vulkan/vulkan.hpp>
#include "App.h"
#include "window/Window.h"

#include "core/Instance.h"
#include "core/Device.h"
#include "core/RenderContext.h"
#include "core/Uniform.h"
#include "core/Queue.h"

#include "gui/GUI.h"
#include "gui/InputCallback.h"
#include "gui/GraphicsCamera.h"

#include "util/VkError.h"
#include "util/VkLogger.h"
#include "util/VkSettings.h"

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE;

namespace vkl {

App::App() : name{"App"}, apiVersion{VK_API_VERSION_1_0} {
  spdlog::set_level(spdlog::level::debug);
}

App::~App() {

  device->getVkDevice().waitIdle();

  currCmdFence    = VK_NULL_HANDLE;
  currScSemaphore = {VK_NULL_HANDLE, VK_NULL_HANDLE};

  camUniform.reset();

  if (vkDescPool) {
    device->getVkDevice().destroyDescriptorPool(vkDescPool);
  }

  //if (camDescLayout) {
  //  device->getVkDevice().destroyDescriptorSetLayout(camDescLayout);
  //}

  //if (vkDescriptorPool) {
  //  device->getVkDevice().destroyDescriptorPool(vkDescriptorPool);
  //}

  //for (auto& camUB : camUBs) {
  //  camUB.clear();
  //}

  gui.reset();

  for (auto& frameBuffer : vkFramebuffers) {
    device->getVkDevice().destroyFramebuffer(frameBuffer);
  }
  vkFramebuffers.clear();

  if (vkRenderPass) {
    device->getVkDevice().destroyRenderPass(vkRenderPass);
  }

  renderContext.reset();
  device.reset();

  if (vkSurface) {
    instance->getVkInstance().destroySurfaceKHR(vkSurface);
  }

  instance.reset();
}

void App::registerWindow(std::unique_ptr<Window>& _window) {
  window = std::move(_window);
}

void App::onWindowResized(int w, int h) {
  if (!renderContext->getVkSwapchain()) {
    LOGW("Can't handle surface changes in headless mode, skipping.");
    return;
  }

  vk::SurfaceCapabilitiesKHR surfaceProps
      = device->getVkPhysicalDevice().getSurfaceCapabilitiesKHR(vkSurface);

  if (surfaceProps.currentExtent.width == 0xFFFFFFFF) {
    LOGE("Surface Width is 0 ");
    return;
  }

  auto& prevExtent = renderContext->getSwapchainProps().extent;
  //Only recreate the swapchain if the dimensions have changed;
  //handle_surface_changes() is called on VK_SUBOPTIMAL_KHR,
  //which might not be due to a surface resize
  if (surfaceProps.currentExtent.width != prevExtent.width
      || surfaceProps.currentExtent.height != prevExtent.height) {
    //Recreate swapchain
    device->getVkDevice().waitIdle();

    window->updateExtent(surfaceProps.currentExtent.width,
                         surfaceProps.currentExtent.height);
    renderContext->resizeSwapChain();
    createFrameBuffer();

    renderContext->createCommandBuffer(vk::CommandBufferLevel::ePrimary);
    //buildCommandBuffers();
    device->getVkDevice().waitIdle();
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
  createCommandPool();
  createCommandBuffer();
  renderContext->prepare();

  //setup subpass
  createRenderPass();
  createFrameBuffer();

  createGUI();

  prepared = true;
  return prepared;
}

void App::run() {
  while (!endApp) {
    onRender();
  }
}

void App::end() {}

void App::createInstance() {
  VkSettings::availableInstanceExtProps = vk::enumerateInstanceExtensionProperties();

  auto& properties = VkSettings::availableInstanceExtProps;

  //get the surface extensions
  auto surfExtensions = window->getRequiredSurfaceExtension();
  for (auto& surfExtension : surfExtensions) {
    VkSettings::addInstanceExtension(surfExtension, properties);
  }

#ifdef VKL_DEBUG

  VkSettings::hasDebugUtil
      = VkSettings::addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, properties);
  bool debugReport{false};
  if (!VkSettings::hasDebugUtil)
    debugReport = VkSettings::addInstanceExtension(VK_EXT_DEBUG_REPORT_EXTENSION_NAME,
                                                   properties);

  if (!VkSettings::hasDebugUtil && !debugReport) {
    LOGW("Neither of {} or {} are available; disabling debug reporting",
         VK_EXT_DEBUG_UTILS_EXTENSION_NAME,
         VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
  }
#endif
  bool& headless = VkSettings::headless;
  headless       = window->getWindowInfo().mode == WindowInfo::Mode::Headless;
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

    VkSettings::addLayer(VK_LAYER_KHRONOS_VALIDATION_NAME, VkSettings::availableLayers);
  }
#endif

  instance = std::make_unique<Instance>(name,
                                        VkSettings::enabledInstanceExtensions,
                                        VkSettings::enabledLayers,
                                        apiVersion);
  if (!instance) {
    LOGE("failed to init instance");
    return;
  }
}

void App::createDevice() {

  vkSurface      = window->createSurface(instance.get());
  bool& headless = VkSettings::headless;

  if (!vkSurface && !headless)
    throw std::runtime_error("Failed to create window surface.");

  auto deviceCands = instance->getVkInstance().enumeratePhysicalDevices();
  if (deviceCands.empty()) {
    LOGE("Could not find a physical device");
    throw std::runtime_error("Could not find a physical device");
  }

  vk::PhysicalDevice vkPhysicalDevice = deviceCands[0];
  for (auto& device : deviceCands) {
    if (device.getProperties().deviceType == vk::PhysicalDeviceType::eDiscreteGpu) {

      int queueCount = device.getQueueFamilyProperties().size();
      for (uint32_t queue_idx = 0; static_cast<size_t>(queue_idx) < queueCount;
           queue_idx++) {

        if (device.getSurfaceSupportKHR(queue_idx, vkSurface)) {
          vkPhysicalDevice = device;
          break;
        }
      }
    }
  }

  device
      = std::make_unique<Device>(instance->getVkInstance(), vkPhysicalDevice, vkSurface);

  VkSettings::availableDeviceExtensions
      = vkPhysicalDevice.enumerateDeviceExtensionProperties();

  auto& properties = VkSettings::availableDeviceExtensions;

  bool instanceEnabledHeadless
      = VkSettings::isInInstanceExtension(VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME);

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
    auto debugExtensionIt
        = std::find_if(properties.begin(),
                       properties.end(),
                       [](vk::ExtensionProperties const& ep) {
                         return strcmp(ep.extensionName,
                                       VK_EXT_DEBUG_MARKER_EXTENSION_NAME)
                                == 0;
                       });
    if (debugExtensionIt != properties.end()) {
      LOGI("Vulkan debug utils enabled ({})", VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
      //TODO: debug marker
      //debug_utils =
      //std::make_unique<vkb::core::HPPDebugMarkerExtDebugUtils>();
      //add_device_extension(VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
    }
  }

#endif

  device->initLogicalDevice();
}

void App::createRenderContext() {

#ifdef VK_USE_PLATFORM_ANDROID_KHR
  vk::PresentModeKHR present_mode = (window->get_properties().vsync == Window::Vsync::OFF)
                                        ? vk::PresentModeKHR::eMailbox
                                        : vk::PresentModeKHR::eFifo;
  std::vector<vk::PresentModeKHR> present_mode_priority_list{
      vk::PresentModeKHR::eFifo,
      vk::PresentModeKHR::eMailbox,
      vk::PresentModeKHR::eImmediate};
#else
  vk::PresentModeKHR present_mode
      = (window->getWindowInfo().vsync == WindowInfo::Vsync::ON)
            ? vk::PresentModeKHR::eFifo
            : vk::PresentModeKHR::eMailbox;

  VkSettings::presentModePriorities = {vk::PresentModeKHR::eMailbox,
                                       vk::PresentModeKHR::eFifo,
                                       vk::PresentModeKHR::eImmediate};
#endif

  VkSettings::surfaceFormatPriorities
      = {/*{vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear},*/
         {vk::Format::eR8G8B8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear},
         {vk::Format::eB8G8R8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear}};

  renderContext
      = std::make_unique<RenderContext>(device.get(), window.get(), present_mode);

  if (!renderContext) throw std::runtime_error("cannot create render context");
}

void App::createCommandPool() {
  renderContext->createCommandPool(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
}

void App::createCommandBuffer() {
  renderContext->createCommandBuffer(vk::CommandBufferLevel::ePrimary);
}

void App::initializeSwapchainFrames() {
  renderContext->initializeSwapchainImages();
}

void App::createRenderPass() {
  auto& scprops       = renderContext->getSwapchainProps();
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

  vkRenderPass = device->getVkDevice().createRenderPass(renderPassCI);
}

void App::createFrameBuffer() {

  if (!vkFramebuffers.empty()) {
    for (auto& framebuffer : vkFramebuffers) {
      device->getVkDevice().destroyFramebuffer(framebuffer);
    }
    vkFramebuffers.clear();
  }

  auto swapSize = renderContext->getSwapchainProps().imageCount;
  auto extent   = renderContext->getSwapchainProps().extent;

  std::array<vk::ImageView, 2> attachments;
  attachments[1] = renderContext->getDepthStencilImage().vkImageView;

  vk::FramebufferCreateInfo framebufferCI({},
                                          vkRenderPass,
                                          attachments,
                                          extent.width,
                                          extent.height,
                                          1);

  auto& swapchainImages = renderContext->getSwapchainImages();

  vkFramebuffers.reserve(swapSize);

  for (auto& image : swapchainImages) {
    attachments[0] = image.vkImageView;
    vkFramebuffers.push_back(device->getVkDevice().createFramebuffer(framebufferCI));
  }
}

void App::createGUI() {
  gui = std::make_unique<GUI>();
  gui->initialize(device.get(), renderContext.get(), VK_NULL_HANDLE);
  gui->prepare(vkRenderPass);

  inputCallback    = std::make_unique<InputCallback>();
  auto keyCallback = [&](int key) {
    if (key == 526) {
      this->endApp = true;
    }
  };
  inputCallback->registerKeyPressed(std::move(keyCallback));

  gui->addInputCallback(inputCallback.get());

  createGraphicsCamera();
}

void App::createGraphicsCamera() {
  auto& extent   = renderContext->getSwapchainProps().extent;
  graphicsCamera = std::make_unique<GraphicsCamera>(GraphicsCamera::VULKAN,
                                                    extent.width,
                                                    extent.height,
                                                    GraphicsCamera::PERSPECTIVE);

  gui->addInputCallback(graphicsCamera.get());

  auto          count = renderContext->getSwapchainImageCount();
  CameraUniform tmp;
  camUniform
      = std::make_unique<Uniform>(device.get(), count, &tmp, sizeof(CameraUniform));

  vk::DescriptorSetLayoutBinding camBinding{0,
                                            vk::DescriptorType::eUniformBuffer,
                                            1,
                                            vk::ShaderStageFlagBits::eVertex};

  auto                         camUniCount = camUniform->getBufferCount();
  vk::DescriptorPoolSize       poolSize{vk::DescriptorType::eUniformBuffer, camUniCount};
  vk::DescriptorPoolCreateInfo descPoolCI({}, camUniCount, poolSize);

  auto& descPool = gui->getVkDescPool();
  camUniform->createDescSets(camBinding, descPool);
  for (size_t i = 0; i < count; i++) {
    updateCameraUniform(i);
  }
}

void App::createVkDescriptorPool() {
  std::vector<vk::DescriptorPoolSize> sizes
      = {{vk::DescriptorType::eSampler, 100},
         {vk::DescriptorType::eCombinedImageSampler, 100},
         {vk::DescriptorType::eSampledImage, 100},
         {vk::DescriptorType::eStorageImage, 100},
         {vk::DescriptorType::eUniformTexelBuffer, 100},
         {vk::DescriptorType::eStorageTexelBuffer, 100},
         {vk::DescriptorType::eUniformBuffer, 100},
         {vk::DescriptorType::eStorageBuffer, 100},
         {vk::DescriptorType::eUniformBufferDynamic, 100},
         {vk::DescriptorType::eStorageBufferDynamic, 100},
         {vk::DescriptorType::eInputAttachment, 100}};

  auto                         maxSets = 100 * sizes.size();
  vk::DescriptorPoolCreateInfo descPoolCI({}, maxSets, sizes);

  vkDescPool = device->getVkDevice().createDescriptorPool(descPoolCI);
}

void App::requestGpuFeatures(Device* vkPhysicalDevice) {
  LOGD("This Function should be overriden by app");
}

void App::buildCommandBuffer() {}

vk::CommandBuffer App::beginCommandBuffer() {
  //if you dont want to recycle
  vk::CommandBufferBeginInfo cmdBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  //if you want to recycle
  //vk::CommandBufferBeginInfo cmdBeginInfo;

  std::vector<vk::ClearValue> clearValues
      = {{vk::ClearColorValue({0.10f, 0.10f, 0.10f, 1.0f}),
          vk::ClearDepthStencilValue(1.0f, 0)}};

  auto& renderCmdBuffers = renderContext->getRenderCommandBuffers();
  auto& swapChainImages  = renderContext->getSwapchainImages();
  auto  cmdSize          = renderCmdBuffers.size();

  uint32_t queFamilyIdx = renderContext->getQueue()->getFamilyIdx();

  vk::CommandBuffer& renderCmdBuffer = renderCmdBuffers[currSwapchainIdx];
  vk::Image&         swapChainImage  = swapChainImages[currSwapchainIdx].vkImage;

  renderCmdBuffer.reset();
  renderCmdBuffer.begin(cmdBeginInfo);
  vk::RenderPassBeginInfo renderBeginInfo(vkRenderPass,
                                          {},
                                          {{}, renderContext->getSwapchainProps().extent},
                                          clearValues);

  renderBeginInfo.framebuffer = vkFramebuffers[currSwapchainIdx];
  renderCmdBuffer.beginRenderPass(renderBeginInfo, vk::SubpassContents::eInline);

  auto extent = renderContext->getSwapchainProps().extent;

  vk::Viewport viewport;
  viewport.width    = extent.width;
  viewport.height   = extent.height;
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;

  vk::Rect2D scissor({}, extent);

  renderCmdBuffer.setViewport(0, viewport);
  renderCmdBuffer.setScissor(0, scissor);

  return renderCmdBuffer;
}

void App::updateCameraUniform(int idx) {
  auto& camUniformData = graphicsCamera->cam;
  auto  memSize        = sizeof(CameraUniform);
  camUniform->update(idx, &camUniformData);

  //std::cout << camUniformData.M <<"\n"<< std::endl;
  //std::cout << camUniformData.V << "\n" << std::endl;
  //std::cout << camUniformData.P << "\n" << std::endl;
  //auto vec = Eigen::Vector4f(0.0, -0.5, 0.0, 1.0);
  //std::cout << camUniformData.P * camUniformData.V * camUniformData.M * vec <<
  //std::endl; std::cout << std::endl;
}

void App::updateUniform(int idx) {}

void App::onRender() {}

void App::prepareFrame() {

  auto vkSwapchain = renderContext->getVkSwapchain();
  if (!vkSwapchain) return;

  currScSemaphore = renderContext->getCurrSwapchainSemaphore();

  auto vkDevice = device->getVkDevice();

  auto result = vkDevice.acquireNextImageKHR(vkSwapchain,
                                             0xFFFFFFFFFFFFFFFF,
                                             currScSemaphore.available);
  VK_CHECK_ERROR(static_cast<VkResult>(result.result), "acquireNextImageKHR");

  currSwapchainIdx = result.value;

  currCmdFence = renderContext->getCurrCmdFence();

  if (vkDevice.getFenceStatus(currCmdFence) == vk::Result::eNotReady) {
    vkDevice.waitForFences({currCmdFence}, VK_TRUE, UINT64_MAX);
  }
  vkDevice.resetFences({currCmdFence});
}

void App::presentFrame() {

  auto& vkQueue   = renderContext->getVkQueue();
  auto& cmdBuffer = renderContext->getRenderCommandBuffers()[currSwapchainIdx];

  vk::PipelineStageFlags waitStageMask
      = vk::PipelineStageFlagBits::eColorAttachmentOutput;

  vk::SubmitInfo submitInfo(currScSemaphore.available,
                            waitStageMask,
                            cmdBuffer,
                            currScSemaphore.rendered);

  vkQueue.submit(submitInfo, currCmdFence);
  auto vkSwapchain = renderContext->getVkSwapchain();

  vk::PresentInfoKHR presentInfo(currScSemaphore.rendered, vkSwapchain, currSwapchainIdx);
  vkQueue.presentKHR(presentInfo);
}

}  //namespace vkl