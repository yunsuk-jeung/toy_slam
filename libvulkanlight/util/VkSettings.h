#pragma once

#include <vector>
#include <string>
#include <vulkan/vulkan.hpp>

//#define VK_EXT_DEBUG_UTILS_EXTENSION_NAME "VK_EXT_debug_utils"
//#define VK_EXT_DEBUG_REPORT_EXTENSION_NAME "VK_EXT_debug_report"
#define VK_LAYER_KHRONOS_VALIDATION_NAME "VK_LAYER_KHRONOS_validation"
//#define VK_EXT_HEADLESS_SURFACE_EXTENSION_NAME "VK_EXT_headless_surface"
//#define VK_KHR_SURFACE_EXTENSION_NAME "VK_KHR_surface"
//#define VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME \
//  "VK_KHR_get_physical_device_properties2"

namespace vkl {
class VkSettings {
public:
  static bool
  addInstanceExtension(const char*                                 requiredExtName,
                       const std::vector<vk::ExtensionProperties>& available_exts);

  static bool addLayer(const char*                             requiredLayerName,
                       const std::vector<vk::LayerProperties>& availableLayers);

  static bool isAvailableDeviceExtension(const char* requiredExtName);
  static bool addDeviceExtension(const char* requiredExtName);

  static bool
  addDeviceExtension(const char*                                 requiredExtName,
                     const std::vector<vk::ExtensionProperties>& available_exts);

  static bool       isInInstanceExtension(const char* queryName);
  static vk::Format getSuitableDepthFormat(vk::PhysicalDevice device);

  static bool isInDeviceExtension(const char* queryName);

public:
  //############# should set for app ################//
  static bool headless;
  static bool hasDebugUtil;

  static std::vector<vk::Format> depthFormatPriorities;

  //############## creating vkInstance ##############//

  static std::vector<vk::ExtensionProperties> availableInstanceExtProps;
  static std::vector<const char*>             enabledInstanceExtensions;

  static std::vector<vk::LayerProperties> availableLayers;
  static std::vector<const char*>         enabledLayers;

  //############## creating vkDevice ##############//
  static std::vector<vk::ExtensionProperties> availableDeviceExtensions;
  static std::vector<const char*>             enabledDeviceExtensions;

  static bool           highPriorityGraphicsQueue;
  static vk::QueueFlags queuryQueueTypes;

  //############## creating RenderContext #############//
  static std::vector<vk::PresentModeKHR>   presentModePriorities;
  static std::vector<vk::SurfaceFormatKHR> surfaceFormatPriorities;

  static vk::SurfaceCapabilitiesKHR surfaceCapabilities;

  static std::vector<vk::SurfaceFormatKHR> availableSurfaceFormats;
  static std::vector<vk::PresentModeKHR>   availablePresentModes;
};
}  //namespace vkl