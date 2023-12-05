#pragma once

#include <vector>
#include <string>
#include <vulkan/vulkan.hpp>

#define VK_CHECK_ERROR(ans, message)                                                     \
  { VkError::vkAssert((ans), message, __FILE__, __LINE__); };

#define VK_CHECK_ERROR2(ans, message)                                                    \
  { VkError::vkAssert2((ans), message, __FILE__, __LINE__); };

class VkError {
public:
  static void vkAssert(VkResult    result,
                       const char* message,
                       const char* file,
                       int         line,
                       bool        abort = true);

  static void vkAssert2(vk::Result  result,
                        const char* message,
                        const char* file,
                        int         line,
                        bool        abort = true);
};