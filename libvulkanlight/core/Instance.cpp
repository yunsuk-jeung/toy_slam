#pragma once

#include <iostream>
#include <volk.h>
#include "Instance.h"
#include "VklLogger.h"
#include "VkSettings.h"

namespace vkl {
namespace {
#if defined(VKL_DEBUG) || defined(VKL_VALIDATION_LAYERS)

VKAPI_ATTR VkBool32 VKAPI_CALL
debug_utils_messenger_callback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
                               VkDebugUtilsMessageTypeFlagsEXT        message_type,
                               const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
                               void*                                       user_data) {
  //Log debug message
  if (message_severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    VklLogW("{} - {}: {}",
            callback_data->messageIdNumber,
            callback_data->pMessageIdName,
            callback_data->pMessage);
  }
  else if (message_severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    VklLogE("{} - {}: {}",
            callback_data->messageIdNumber,
            callback_data->pMessageIdName,
            callback_data->pMessage);
  }
  return VK_FALSE;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(VkDebugReportFlagsEXT flags,
                                                     VkDebugReportObjectTypeEXT /*type*/,
                                                     uint64_t /*object*/,
                                                     size_t /*location*/,
                                                     int32_t /*message_code*/,
                                                     const char* layer_prefix,
                                                     const char* message,
                                                     void* /*user_data*/) {
  if (flags & VK_DEBUG_REPORT_ERROR_BIT_EXT) { VklLogE("{}: {}", layer_prefix, message); }
  else if (flags & VK_DEBUG_REPORT_WARNING_BIT_EXT) {
    VklLogW("{}: {}", layer_prefix, message);
  }
  else if (flags & VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT) {
    VklLogW("{}: {}", layer_prefix, message);
  }
  else { VklLogI("{}: {}", layer_prefix, message); }
  return VK_FALSE;
}
#endif
}  //namespace

Instance::Instance(const std::string&              name,
                   const std::vector<const char*>& enabledExts,
                   const std::vector<const char*>& enabledLayers,
                   uint32_t                        apiVersion) {
  auto& availableExtensions = VkSettings::availableInstanceExtProps;

  VkSettings::addInstanceExtension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME,
                                   availableExtensions);

  vk::ApplicationInfo appInfo{name.c_str(), 0, "Vulkan App", 0, apiVersion};

  vk::InstanceCreateInfo instance_info({}, &appInfo, enabledLayers, enabledExts);

  bool& debugEnabled = VkSettings::hasDebugUtil;

#if defined(VKL_DEBUG) || defined(VKL_VALIDATION_LAYERS)
  vk::DebugUtilsMessengerCreateInfoEXT debug_utils_create_info;
  vk::DebugReportCallbackCreateInfoEXT debug_report_create_info;
  if (debugEnabled) {
    debug_utils_create_info = vk::DebugUtilsMessengerCreateInfoEXT(
      {},
      vk::DebugUtilsMessageSeverityFlagBitsEXT::eError
        | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning,
      vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation
        | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
      debug_utils_messenger_callback);

    instance_info.pNext = &debug_utils_create_info;
  }
  else {
    debug_report_create_info = vk::DebugReportCallbackCreateInfoEXT(
      vk::DebugReportFlagBitsEXT::eError | vk::DebugReportFlagBitsEXT::eWarning
        | vk::DebugReportFlagBitsEXT::ePerformanceWarning,
      debug_callback);

    instance_info.pNext = &debug_report_create_info;
  }
#endif

  vkInstance = vk::createInstance(instance_info);

  if (!vkInstance) {
    VklLogW("Failed to create vkInstance");
    throw std::runtime_error("Failted to create vkInstance");
  }

  VULKAN_HPP_DEFAULT_DISPATCHER.init(vkInstance);

  volkLoadInstance(vkInstance);

#if defined(VKL_DEBUG) || defined(VKL_VALIDATION_LAYERS)
  if (debugEnabled) {
    debugUtilsmessenger = vkInstance.createDebugUtilsMessengerEXT(
      debug_utils_create_info);
  }
  else {
    debugReportCallback = vkInstance.createDebugReportCallbackEXT(
      debug_report_create_info);
  }
#endif
}

Instance::~Instance() {
#if defined(VKL_DEBUG) || defined(VKL_VALIDATION_LAYERS)
  if (debugUtilsmessenger) {
    vkInstance.destroyDebugUtilsMessengerEXT(debugUtilsmessenger);
    debugUtilsmessenger = VK_NULL_HANDLE;
  }
  if (debugReportCallback) {
    vkInstance.destroyDebugReportCallbackEXT(debugReportCallback);
    debugReportCallback = VK_NULL_HANDLE;
  }
#endif

  vkInstance.destroy();
}

}  //namespace vkl