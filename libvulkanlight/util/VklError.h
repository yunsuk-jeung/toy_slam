#pragma once

#include <string>
#include <vector>
#include <vulkan/vulkan.hpp>
#include "VklLogger.h"

#define VKL_CHECK_ERROR(ans, message)                                                    \
  { vkl::Error::vkAssert((ans), message, __FILE__, __LINE__); };

#define VKL_CHECK_ERROR2(ans, message)                                                   \
  { vkl::Error::vkAssert2((ans), message, __FILE__, __LINE__); };

#ifdef VKL_DEBUG
#define VKL_ASSERT(result)                                                               \
  vkl::Error::Assert(!!(result), __FILE__, __LINE__, __FUNCTION__)
#define VKL_ASSERT_MESSAGE(result, message)                                              \
  vkl::Error::Assert(!!(result), message, __FILE__, __LINE__, __FUNCTION__)
#else
#define VKL_ASSERT(result)
#define VKL_ASSERT_MESSAGE(result, message)
#endif
namespace vkl {
class Error {
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

  static void Assert(bool result, const char* file, int line, const char* function) {
    if (result) {
      return;
    }
    VklLogger::logE(LogUtil::extractFileName(file),
                    line,
                    "Assertion failed in {}",
                    function);
    std::abort();
  }
  static void Assert(bool        result,
                     const char* message,
                     const char* file,
                     int         line,
                     const char* function) {
    if (result) {
      return;
    }
    VklLogger::logE(LogUtil::extractFileName(file),
                    line,
                    "Assertion failed in {} by {}",
                    function,
                    message);
    std::abort();
  }
};
}  //namespace vkl