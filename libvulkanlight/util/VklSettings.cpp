#include "VklSettings.h"

namespace vkl {

//############# should set for app ################//

bool VklSettings::headless{false};
bool VklSettings::hasDebugUtil{false};

std::vector<vk::Format> VklSettings::depthFormatPriorities{vk::Format::eD32Sfloat,
                                                           vk::Format::eD24UnormS8Uint,
                                                           vk::Format::eD16Unorm};

//############## creating vkInstance ##############
std::vector<vk::ExtensionProperties> VklSettings::availableInstanceExtProps{};
std::vector<const char*>             VklSettings::enabledInstanceExtensions{};

std::vector<vk::LayerProperties> VklSettings::availableLayers{};
std::vector<const char*>         VklSettings::enabledLayers{};
//############## creating Device ##############

std::vector<vk::ExtensionProperties> VklSettings::availableDeviceExtensions{};
std::vector<const char*>             VklSettings::enabledDeviceExtensions{};

bool VklSettings::highPriorityGraphicsQueue{true};

vk::QueueFlags VklSettings::queuryQueueTypes{vk::QueueFlagBits::eGraphics
                                             | vk::QueueFlagBits::eCompute};
//############## creating RenderContext #############

std::vector<vk::PresentModeKHR>   VklSettings::presentModePriorities{};
std::vector<vk::SurfaceFormatKHR> VklSettings::surfaceFormatPriorities{};

vk::SurfaceCapabilitiesKHR VklSettings::surfaceCapabilities{};

std::vector<vk::SurfaceFormatKHR> VklSettings::availableSurfaceFormats{};
std::vector<vk::PresentModeKHR>   VklSettings::availablePresentModes{};

//#################################################################
bool VklSettings::addInstanceExtension(
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

bool VklSettings::addLayer(const char*                             requiredLayerName,
                           const std::vector<vk::LayerProperties>& availableLayers) {
  for (auto& layerIt : availableLayers) {
    if (strcmp(layerIt.layerName, requiredLayerName) == 0) {
      enabledLayers.emplace_back(requiredLayerName);
      return true;
    }
  }
  return false;
}
bool VklSettings::isAvailableDeviceExtension(const char* requiredExtName) {
  for (auto& extIt : availableDeviceExtensions) {
    if (strcmp(extIt.extensionName, requiredExtName) == 0) {
      return true;
    }
  }
  return false;
}

bool VklSettings::addDeviceExtension(const char* requiredExtName) {
  return addDeviceExtension(requiredExtName, availableDeviceExtensions);
}

bool VklSettings::addDeviceExtension(
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

bool VklSettings::isInInstanceExtension(const char* queryName) {
  for (auto& extIt : enabledInstanceExtensions) {
    if (strcmp(extIt, queryName) == 0) {
      return true;
    }
  }
  return false;
}

vk::Format VklSettings::getSuitableDepthFormat(vk::PhysicalDevice device) {
  vk::Format depthFormat{vk::Format::eUndefined};

  return depthFormat;
}

bool VklSettings::isInDeviceExtension(const char* queryName) {
  for (auto& extIt : enabledDeviceExtensions) {
    if (strcmp(extIt, queryName) == 0) {
      return true;
    }
  }
  return false;
}

}  //namespace vkl