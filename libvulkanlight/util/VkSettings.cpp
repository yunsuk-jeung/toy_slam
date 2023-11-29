#include "VkSettings.h"

namespace vkl {

//############# should set for app ################//

bool VkSettings::headless{false};
bool VkSettings::hasDebugUtil{false};

std::vector<vk::Format> VkSettings::depthFormatPriorities{vk::Format::eD32Sfloat,
                                                          vk::Format::eD24UnormS8Uint,
                                                          vk::Format::eD16Unorm};

//############## creating vkInstance ##############
std::vector<vk::ExtensionProperties> VkSettings::availableInstanceExtProps{};
std::vector<const char*>             VkSettings::enabledInstanceExtensions{};

std::vector<vk::LayerProperties> VkSettings::availableLayers{};
std::vector<const char*>         VkSettings::enabledLayers{};
//############## creating Device ##############

std::vector<vk::ExtensionProperties> VkSettings::availableDeviceExtensions{};
std::vector<const char*>             VkSettings::enabledDeviceExtensions{};

bool VkSettings::highPriorityGraphicsQueue{true};

vk::QueueFlags VkSettings::queuryQueueTypes{vk::QueueFlagBits::eGraphics
                                            | vk::QueueFlagBits::eCompute};
//############## creating RenderContext #############

std::vector<vk::PresentModeKHR>   VkSettings::presentModePriorities{};
std::vector<vk::SurfaceFormatKHR> VkSettings::surfaceFormatPriorities{};

vk::SurfaceCapabilitiesKHR VkSettings::surfaceCapabilities{};

std::vector<vk::SurfaceFormatKHR> VkSettings::availableSurfaceFormats{};
std::vector<vk::PresentModeKHR>   VkSettings::availablePresentModes{};

//#################################################################
bool VkSettings::addInstanceExtension(
  const char*                                 requiredExtName,
  const std::vector<vk::ExtensionProperties>& available_exts) {
  for (auto& extIt : available_exts) {
    if (strcmp(extIt.extensionName, requiredExtName) == 0) {
      enabledInstanceExtensions.emplace_back(requiredExtName);
      return true;
    }
  }
  return false;
}

bool VkSettings::addLayer(const char*                             requiredLayerName,
                          const std::vector<vk::LayerProperties>& availableLayers) {
  for (auto& layerIt : availableLayers) {
    if (strcmp(layerIt.layerName, requiredLayerName) == 0) {
      enabledLayers.emplace_back(requiredLayerName);
      return true;
    }
  }
  return false;
}
bool VkSettings::isAvailableDeviceExtension(const char* requiredExtName) {
  for (auto& extIt : availableDeviceExtensions) {
    if (strcmp(extIt.extensionName, requiredExtName) == 0) { return true; }
  }
  return false;
}

bool VkSettings::addDeviceExtension(const char* requiredExtName) {
  return addDeviceExtension(requiredExtName, availableDeviceExtensions);
}

bool VkSettings::addDeviceExtension(
  const char*                                 requiredExtName,
  const std::vector<vk::ExtensionProperties>& available_exts) {
  for (auto& extIt : available_exts) {
    if (strcmp(extIt.extensionName, requiredExtName) == 0) {
      enabledDeviceExtensions.emplace_back(requiredExtName);
      return true;
    }
  }
  return false;
}

bool VkSettings::isInInstanceExtension(const char* queryName) {
  for (auto& extIt : enabledInstanceExtensions) {
    if (strcmp(extIt, queryName) == 0) { return true; }
  }
  return false;
}

vk::Format VkSettings::getSuitableDepthFormat(vk::PhysicalDevice device) {
  vk::Format depthFormat{vk::Format::eUndefined};

  return depthFormat;
}

bool VkSettings::isInDeviceExtension(const char* queryName) {
  for (auto& extIt : enabledDeviceExtensions) {
    if (strcmp(extIt, queryName) == 0) { return true; }
  }
  return false;
}

}  //namespace vkl