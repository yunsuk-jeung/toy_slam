#pragma once
#include <memory>
#include "Logger.h"

#define __VKL_FILENAME__ LogUtil::extractFileName(__FILE__)

namespace vkl {

class VklLogger {
public:
  static std::shared_ptr<spdlog::logger> logger;
  static void                            init();

  template <typename... Args>
  static void logI(const char* fmt, Args... args) {
    logger->info(fmt, args...);
  }
  template <typename... Args>
  static void logW(const char* fmt, Args... args) {
    logger->warn(fmt, args...);
  }
  template <typename... Args>
  static void logD(const char* fmt, Args... args) {
    logger->debug(fmt, args...);
  }

  template <typename... Args>
  static void logE(const char* fmt, Args... args) {
    logger->error("[{}:{}] {}", __VKL_FILENAME__, __LINE__, fmt::format(fmt, args...));
  }

};
}  //namespace vkl

#define VklLogD(fmt, ...) VklLogger::logI(fmt, ##__VA_ARGS__);
#define VklLogI(fmt, ...) VklLogger::logW(fmt, ##__VA_ARGS__);
#define VklLogW(fmt, ...) VklLogger::logE(fmt, ##__VA_ARGS__);
#define VklLogE(fmt, ...) VklLogger::logD(fmt, ##__VA_ARGS__);