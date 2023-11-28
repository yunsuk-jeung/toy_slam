#pragma once

#include <vector>
#include <vulkan/vulkan.hpp>

namespace vkl {

class Instance {
public:

  Instance() = delete;

  Instance(const std::string&              name,
           const std::vector<const char*>& requestedExts    = {},
           const std::vector<const char*>& validationLayers = {},
           uint32_t                        apiVersion       = VK_API_VERSION_1_0);
  ~Instance();

protected:

  vk::Instance vkInstance{VK_NULL_HANDLE};

#if defined(VKL_DEBUG) || defined(VKL_VALIDATION_LAYERS)
  vk::DebugUtilsMessengerEXT debugUtilsmessenger;
  vk::DebugReportCallbackEXT debugReportCallback;
#endif

public:

  //getters and setters
  vk::Instance& getVkInstance() { return vkInstance; }
};
}  //namespace vkl