#pragma once

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include "spdlog/sinks/android_sink.h"
#define LOGGER_FORMAT "[%^%l%$] %v"
#define PROJECT_NAME "VULKAN_LIGHT"

#define __VKL_FILENAME__ vkl::LogUtil::extractFileName(__FILE__)

namespace vkl {

class LogUtil {
public:
  static const char* extractFileName(const char* filePath) {
#ifdef _WIN32
    const char* lastSlash = strrchr(filePath, '\\');
#else
    const char* lastSlash = strrchr(filePath, '/');
#endif
    if (lastSlash != nullptr) { return lastSlash + 1; }
    else { return filePath; }
  }
};
}  //namespace vkl

#define VklLogD(...) spdlog::debug(__VA_ARGS__);
#define VklLogI(...) spdlog::info(__VA_ARGS__);
#define VklLogW(...) spdlog::warn(__VA_ARGS__);
#define VklLogE(...)                                                                     \
  spdlog::error("[{}:{}] {}", __VKL_FILENAME__, __LINE__, fmt::format(__VA_ARGS__));
