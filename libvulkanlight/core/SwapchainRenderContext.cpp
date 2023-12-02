#include "SwapchainRenderContext.h"
#include "Device.h"
#include "window/Window.h"
#include "VkSettings.h"
#include "VklLogger.h"

namespace vkl {
namespace {
template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : ((hi < v) ? hi : v);
}

vk::Extent2D chooseExtent(vk::Extent2D        request_extent,
                          const vk::Extent2D& min_image_extent,
                          const vk::Extent2D& max_image_extent,
                          const vk::Extent2D& current_extent) {
  if (current_extent.width == 0xFFFFFFFF) { return request_extent; }

  if (request_extent.width < 1 || request_extent.height < 1) {
    VklLogW("(HPPSwapchain) Image extent ({}, {}) not supported. Selecting ({}, "
            "{}).",
            request_extent.width,
            request_extent.height,
            current_extent.width,
            current_extent.height);
    return current_extent;
  }

  request_extent.width  = clamp(request_extent.width,
                               min_image_extent.width,
                               max_image_extent.width);
  request_extent.height = clamp(request_extent.height,
                                min_image_extent.height,
                                max_image_extent.height);

  return request_extent;
}

vk::PresentModeKHR choosePresentMode(
  vk::PresentModeKHR                     request_present_mode,
  const std::vector<vk::PresentModeKHR>& available_present_modes,
  const std::vector<vk::PresentModeKHR>& present_mode_priority_list,
  bool                                   log) {
  //Try to find the requested present mode in the available present modes
  auto const present_mode_it = std::find(available_present_modes.begin(),
                                         available_present_modes.end(),
                                         request_present_mode);
  if (present_mode_it == available_present_modes.end()) {
    //If the requested present mode isn't found, then try to find a mode from
    //the priority list
    auto const chosen_present_mode_it =
      std::find_if(present_mode_priority_list.begin(),
                   present_mode_priority_list.end(),
                   [&available_present_modes](vk::PresentModeKHR present_mode) {
                     return std::find(available_present_modes.begin(),
                                      available_present_modes.end(),
                                      present_mode)
                            != available_present_modes.end();
                   });

    //If nothing found, always default to FIFO
    vk::PresentModeKHR const chosen_present_mode = (chosen_present_mode_it
                                                    != present_mode_priority_list.end())
                                                     ? *chosen_present_mode_it
                                                     : vk::PresentModeKHR::eFifo;

    VklLogW("(HPPSwapchain) Present mode '{}' not supported. Selecting '{}'.",
            vk::to_string(request_present_mode),
            vk::to_string(chosen_present_mode));
    return chosen_present_mode;
  }
  else {
    if (log)
      VklLogI("(HPPSwapchain) Present mode selected: {}",
              to_string(request_present_mode));
    return request_present_mode;
  }
}

vk::SurfaceFormatKHR chooseSurfaceFormat(
  const vk::SurfaceFormatKHR               requested_surface_format,
  const std::vector<vk::SurfaceFormatKHR>& available_surface_formats,
  const std::vector<vk::SurfaceFormatKHR>& surface_format_priority_list,
  bool                                     log) {
  //Try to find the requested surface format in the available surface formats
  auto const surface_format_it = std::find(available_surface_formats.begin(),
                                           available_surface_formats.end(),
                                           requested_surface_format);

  //If the requested surface format isn't found, then try to request a format
  //from the priority list
  if (surface_format_it == available_surface_formats.end()) {
    auto const chosen_surface_format_it =
      std::find_if(surface_format_priority_list.begin(),
                   surface_format_priority_list.end(),
                   [&available_surface_formats](vk::SurfaceFormatKHR surface_format) {
                     return std::find(available_surface_formats.begin(),
                                      available_surface_formats.end(),
                                      surface_format)
                            != available_surface_formats.end();
                   });

    //If nothing found, default to the first available format
    vk::SurfaceFormatKHR const& chosen_surface_format = (chosen_surface_format_it
                                                         != surface_format_priority_list
                                                              .end())
                                                          ? *chosen_surface_format_it
                                                          : available_surface_formats[0];

    VklLogW("(Swapchain) Surface format ({}) not supported. Selecting ({}).",
            vk::to_string(requested_surface_format.format) + ", "
              + vk::to_string(requested_surface_format.colorSpace),
            vk::to_string(chosen_surface_format.format) + ", "
              + vk::to_string(chosen_surface_format.colorSpace));
    return chosen_surface_format;
  }
  else {
    if (log)
      VklLogI("(Swapchain) Surface format selected: {}",
              vk::to_string(requested_surface_format.format) + ", "
                + vk::to_string(requested_surface_format.colorSpace));
    return requested_surface_format;
  }
}

vk::SurfaceTransformFlagBitsKHR chooseTransform(
  vk::SurfaceTransformFlagBitsKHR request_transform,
  vk::SurfaceTransformFlagsKHR    supported_transform,
  vk::SurfaceTransformFlagBitsKHR current_transform) {
  if (request_transform & supported_transform) { return request_transform; }

  VklLogW("(HPPSwapchain) Surface transform '{}' not supported. Selecting '{}'.",
          vk::to_string(request_transform),
          vk::to_string(current_transform));
  return current_transform;
}

vk::CompositeAlphaFlagBitsKHR chooseCompositeAlpha(
  vk::CompositeAlphaFlagBitsKHR request_composite_alpha,
  vk::CompositeAlphaFlagsKHR    supported_composite_alpha) {
  if (request_composite_alpha & supported_composite_alpha) {
    return request_composite_alpha;
  }

  static const std::vector<vk::CompositeAlphaFlagBitsKHR> composite_alpha_priority_list =
    {vk::CompositeAlphaFlagBitsKHR::eOpaque,
     vk::CompositeAlphaFlagBitsKHR::ePreMultiplied,
     vk::CompositeAlphaFlagBitsKHR::ePostMultiplied,
     vk::CompositeAlphaFlagBitsKHR::eInherit};

  auto const chosen_composite_alpha_it = std::find_if(
    composite_alpha_priority_list.begin(),
    composite_alpha_priority_list.end(),
    [&supported_composite_alpha](vk::CompositeAlphaFlagBitsKHR composite_alpha) {
      return composite_alpha & supported_composite_alpha;
    });
  if (chosen_composite_alpha_it == composite_alpha_priority_list.end()) {
    throw std::runtime_error("No compatible composite alpha found.");
  }
  else {
    VklLogW("(HPPSwapchain) Composite alpha '{}' not supported. Selecting '{}.",
            vk::to_string(request_composite_alpha),
            vk::to_string(*chosen_composite_alpha_it));
    return *chosen_composite_alpha_it;
  }
}

bool validateFormatFeature(vk::ImageUsageFlagBits image_usage,
                           vk::FormatFeatureFlags supported_features) {
  return (image_usage != vk::ImageUsageFlagBits::eStorage)
         || (supported_features & vk::FormatFeatureFlagBits::eStorageImage);
}

std::set<vk::ImageUsageFlagBits> chooseImageUsage(
  const std::set<vk::ImageUsageFlagBits>& requested_image_usage_flags,
  vk::ImageUsageFlags                     supported_image_usage,
  vk::FormatFeatureFlags                  supported_features,
  bool                                    log) {
  std::set<vk::ImageUsageFlagBits> validated_image_usage_flags;
  for (auto flag : requested_image_usage_flags) {
    if ((flag & supported_image_usage)
        && validateFormatFeature(flag, supported_features)) {
      validated_image_usage_flags.insert(flag);
    }
    else {
      VklLogW("(HPPSwapchain) Image usage ({}) requested but not supported.",
              vk::to_string(flag));
    }
  }

  if (validated_image_usage_flags.empty()) {
    //Pick the first format from list of defaults, if supported
    static const std::vector<vk::ImageUsageFlagBits> image_usage_priority_list =
      {vk::ImageUsageFlagBits::eColorAttachment,
       vk::ImageUsageFlagBits::eStorage,
       vk::ImageUsageFlagBits::eSampled,
       vk::ImageUsageFlagBits::eTransferDst};

    auto const priority_list_it =
      std::find_if(image_usage_priority_list.begin(),
                   image_usage_priority_list.end(),
                   [&supported_image_usage, &supported_features](auto const image_usage) {
                     return ((image_usage & supported_image_usage)
                             && validateFormatFeature(image_usage, supported_features));
                   });
    if (priority_list_it != image_usage_priority_list.end()) {
      validated_image_usage_flags.insert(*priority_list_it);
    }
  }

  if (validated_image_usage_flags.empty()) {
    throw std::runtime_error("No compatible image usage found.");
  }
  else {
    //Log image usage flags used
    std::string usage_list;
    for (vk::ImageUsageFlagBits image_usage : validated_image_usage_flags) {
      usage_list += to_string(image_usage) + " ";
    }
    if (log) VklLogI("(HPPSwapchain) Image usage flags: {}", usage_list);
  }

  return validated_image_usage_flags;
}

vk::ImageUsageFlags compositeImageFlags(
  std::set<vk::ImageUsageFlagBits>& image_usage_flags) {
  vk::ImageUsageFlags image_usage;
  for (auto flag : image_usage_flags) { image_usage |= flag; }
  return image_usage;
}
}  //namespace
SwapchainRenderContext::SwapchainRenderContext(Device*            _device,
                                               Window*            _window,
                                               vk::PresentModeKHR presentMode)
  : RenderContext(_device, _window, presentMode) {
  queue = &device->getPresentableQueue();

  if (device->getVkSurface()) {
    if (VkSettings::surfaceCapabilities.currentExtent.width == 0xFFFFFFFF) {
      createSwapChain(ctProps.extent);
    }
    else { createSwapChain(); }
  }
}

SwapchainRenderContext::~SwapchainRenderContext() {
  vk::Device vkDevice = device->vk();
  vkDevice.waitIdle();

  if (vkSwapchain) {
    vkDevice.destroySwapchainKHR(vkSwapchain);
    vkSwapchain = VK_NULL_HANDLE;
  }
}

void SwapchainRenderContext::prepare() {
  prepareColor();
  prepareDepthStencil();
}

void SwapchainRenderContext::resizeSwapChain() {
  vk::Extent2D newEx = {window->getWindowInfo().extent.width,
                        window->getWindowInfo().extent.height};

  createSwapChain(newEx,
                  ctProps.imageCount,
                  ctProps.preTransform,
                  vkImageUsageFlags,
                  vkSwapchain);
  prepare();
}

void SwapchainRenderContext::prepareColor() {
  auto vkDevice = device->vk();
  auto vkImages = vkDevice.getSwapchainImagesKHR(vkSwapchain);

  auto imageSize = vkImages.size();

  if (imageSize != ctProps.imageCount) { ctProps.imageCount = imageSize; }
  colorImages.reserve(imageSize);

  vk::ImageViewCreateInfo imageViewCI{};
  imageViewCI.viewType                        = vk::ImageViewType::e2D;
  imageViewCI.format                          = ctProps.surfaceFormat.format;
  imageViewCI.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
  imageViewCI.subresourceRange.baseMipLevel   = 0;
  imageViewCI.subresourceRange.levelCount     = 1;
  imageViewCI.subresourceRange.baseArrayLayer = 0;
  imageViewCI.subresourceRange.layerCount     = 1;

  for (auto& image : vkImages) {
    imageViewCI.image = image;
    colorImages.emplace_back(device, image, imageViewCI);
  }

  if (cmdFences.empty()) {
    vk::FenceCreateInfo fenceInfo(vk::FenceCreateFlagBits::eSignaled);
    cmdFences.reserve(imageSize);
    bfSemaphores.reserve(imageSize);

    for (int i = 0; i < imageSize; ++i) {
      cmdFences.emplace_back(vkDevice.createFence(fenceInfo));
      bfSemaphores.push_back(
        {vkDevice.createSemaphore({}), vkDevice.createSemaphore({})});
    }
  }
}

void SwapchainRenderContext::prepareDepthStencil() {
  ctProps.depthFormat = device->getSuitableDepthFormat();
  auto vkDevice       = device->vk();

  vk::ImageCreateInfo vkImageCI;
  vkImageCI.imageType = vk::ImageType::e2D;
  vkImageCI.format    = ctProps.depthFormat;
  vkImageCI.extent    = vk::Extent3D(ctProps.extent.width, ctProps.extent.height, 1);

  vkImageCI.mipLevels   = 1;
  vkImageCI.arrayLayers = 1;
  vkImageCI.samples     = vk::SampleCountFlagBits::e1;
  vkImageCI.tiling      = vk::ImageTiling::eOptimal;
  vkImageCI.usage       = vk::ImageUsageFlagBits::eDepthStencilAttachment
                    | vk::ImageUsageFlagBits::eTransferSrc;

  vk::ImageViewCreateInfo depthImageViewCI;
  depthImageViewCI.viewType                        = vk::ImageViewType::e2D;
  depthImageViewCI.subresourceRange.baseMipLevel   = 0;
  depthImageViewCI.subresourceRange.levelCount     = 1;
  depthImageViewCI.subresourceRange.baseArrayLayer = 0;
  depthImageViewCI.subresourceRange.layerCount     = 1;
  depthImageViewCI.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eDepth;

  if (vk::Format::eD16UnormS8Uint <= ctProps.depthFormat) {
    depthImageViewCI.subresourceRange.aspectMask |= vk::ImageAspectFlagBits::eStencil;
  }

  depthStencilImages.emplace_back(device,
                                  vkImageCI,
                                  vk::MemoryPropertyFlagBits::eDeviceLocal,
                                  vk::MemoryPropertyFlagBits::eDeviceLocal,
                                  depthImageViewCI);

  /*vk::CommandBufferAllocateInfo info(renderCommandPool,
                                     vk::CommandBufferLevel::ePrimary,
                                     1);

  auto  cmdBuffers = device->vk().allocateCommandBuffers(info);
  auto& cmdBuffer  = cmdBuffers.front();

  vk::CommandBufferBeginInfo cmdBeginIfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
  cmdBuffer.begin(cmdBeginIfo);

  vk::ImageMemoryBarrier barrier({},
                                 {},
                                 vk::ImageLayout::eUndefined,
                                 vk::ImageLayout::eDepthStencilAttachmentOptimal,
                                 queue->getFamilyIdx(),
                                 queue->getFamilyIdx(),
                                 depthStencilImage.vkImage,
                                 depthImageViewCI.subresourceRange);

  cmdBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                            vk::PipelineStageFlagBits::eBottomOfPipe,
                            {},
                            {},
                            {},
                            barrier);
  cmdBuffer.end();
  vk::SubmitInfo submitInfo({}, {}, cmdBuffers, {});
  queue->getVkQueue().submit(submitInfo);
  device->vk().waitIdle();
  device->vk().freeCommandBuffers(renderCommandPool, cmdBuffers);*/
}

void SwapchainRenderContext::createSwapChain(
  vk::Extent2D                     _extent,
  uint32_t                         _imageCount,
  vk::SurfaceTransformFlagBitsKHR  transform,
  std::set<vk::ImageUsageFlagBits> imageUsageFlags,
  vk::SwapchainKHR                 _oldSwapchain) {
  auto& vkPhysicalDevice = device->getVkPhysicalDevice();
  auto& vkDevice         = device->vk();
  auto& vkSurface        = device->getVkSurface();

  setSwapChainProperties(_extent, _imageCount, transform, imageUsageFlags, _oldSwapchain);

  auto& oldSwapchain   = ctProps.oldSwapchain;
  auto& imageCount     = ctProps.imageCount;
  auto& extent         = ctProps.extent;
  auto& surfaceFormat  = ctProps.surfaceFormat;
  auto& arrayLayers    = ctProps.arrayLayers;
  auto& imageUsage     = ctProps.imageUsage;
  auto& preTransform   = ctProps.preTransform;
  auto& compositeAlpha = ctProps.compositeAlpha;
  auto& presentMode    = ctProps.presentMode;
  auto& depthFormat    = ctProps.depthFormat;

  vk::SwapchainCreateInfoKHR const createInfo({},
                                              vkSurface,
                                              imageCount,
                                              surfaceFormat.format,
                                              surfaceFormat.colorSpace,
                                              extent,
                                              arrayLayers,
                                              imageUsage,
                                              {},
                                              {},
                                              preTransform,
                                              compositeAlpha,
                                              presentMode,
                                              {},
                                              oldSwapchain);

  vkSwapchain = vkDevice.createSwapchainKHR(createInfo);

  if (oldSwapchain) {
    device->vk().destroySwapchainKHR(oldSwapchain);
    colorImages.clear();
    depthStencilImages.clear();
  }
}

void SwapchainRenderContext::setSwapChainProperties(
  vk::Extent2D                     _extent,
  uint32_t                         _imageCount,
  vk::SurfaceTransformFlagBitsKHR  transform,
  std::set<vk::ImageUsageFlagBits> IUFlags,
  vk::SwapchainKHR                 _oldSC) {
  auto& vkPhysicalDevice = device->getVkPhysicalDevice();
  auto  vkDevice         = device->vk();
  auto  vkSurface        = device->getVkSurface();

  auto& oldSwapchain   = ctProps.oldSwapchain;
  auto& imageCount     = ctProps.imageCount;
  auto& extent         = ctProps.extent;
  auto& surfaceFormat  = ctProps.surfaceFormat;
  auto& arrayLayers    = ctProps.arrayLayers;
  auto& imageUsage     = ctProps.imageUsage;
  auto& preTransform   = ctProps.preTransform;
  auto& compositeAlpha = ctProps.compositeAlpha;
  auto& presentMode    = ctProps.presentMode;
  auto& depthFormat    = ctProps.depthFormat;

  VkSettings::surfaceCapabilities = vkPhysicalDevice.getSurfaceCapabilitiesKHR(vkSurface);

  auto& presentModePriorities   = VkSettings::presentModePriorities;
  auto& surfaceFormatPriorities = VkSettings::surfaceFormatPriorities;

  VkSettings::availableSurfaceFormats = vkPhysicalDevice.getSurfaceFormatsKHR(vkSurface);

  auto& availableSurfaceFormats = VkSettings::availableSurfaceFormats;

  bool log = false;
  if (!_oldSC) log = true;

  if (log) {
    VklLogI("Surface supports the following surface formats:");
    for (auto& surfaceFormat_ : availableSurfaceFormats) {
      VklLogI("  \t{}",
              vk::to_string(surfaceFormat_.format) + ", "
                + vk::to_string(surfaceFormat_.colorSpace));
    }
  }

  VkSettings::availablePresentModes = vkPhysicalDevice.getSurfacePresentModesKHR(
    vkSurface);
  auto& availablePresentModes = VkSettings::availablePresentModes;

  if (log) {
    VklLogI("Surface supports the following present modes:");
    for (auto& presenMode : availablePresentModes) {
      VklLogI("  \t{}", to_string(presenMode));
    }
  }

  auto& surfaceCapabilities = VkSettings::surfaceCapabilities;

  vk::FormatProperties formatProps = vkPhysicalDevice.getFormatProperties(
    surfaceFormat.format);

  vkImageUsageFlags = chooseImageUsage(IUFlags,
                                       surfaceCapabilities.supportedUsageFlags,
                                       formatProps.optimalTilingFeatures,
                                       log);

  imageCount = clamp(_imageCount,
                     surfaceCapabilities.minImageCount,
                     surfaceCapabilities.maxImageCount ? surfaceCapabilities.maxImageCount
                                                       : 100);

  extent        = chooseExtent(_extent,
                        surfaceCapabilities.minImageExtent,
                        surfaceCapabilities.maxImageExtent,
                        surfaceCapabilities.currentExtent);
  arrayLayers   = 1;
  surfaceFormat = chooseSurfaceFormat(surfaceFormat,
                                      availableSurfaceFormats,
                                      surfaceFormatPriorities,
                                      log);

  imageUsage = compositeImageFlags(this->vkImageUsageFlags);

  preTransform   = chooseTransform(transform,
                                 surfaceCapabilities.supportedTransforms,
                                 surfaceCapabilities.currentTransform);
  compositeAlpha = chooseCompositeAlpha(compositeAlpha,
                                        surfaceCapabilities.supportedCompositeAlpha);

  oldSwapchain = _oldSC;

  //Revalidate the present mode and surface format
  presentMode = choosePresentMode(presentMode,
                                  availablePresentModes,
                                  presentModePriorities,
                                  log);

  surfaceFormat = chooseSurfaceFormat(surfaceFormat,
                                      availableSurfaceFormats,
                                      surfaceFormatPriorities,
                                      log);
}
}  //namespace vkl