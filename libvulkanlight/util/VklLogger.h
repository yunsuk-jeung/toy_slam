#pragma once
#include <memory>
#include "Logger.h"

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
  static void logE(const char* file, int line, const char* fmt, Args... args) {
    logger->error("[{}:{}] {}", file, line, fmt::format(fmt, args...));
  }
};
}  //namespace vkl

#define vklLogD(fmt, ...) VklLogger::logD(fmt, ##__VA_ARGS__);
#define vklLogI(fmt, ...) VklLogger::logI(fmt, ##__VA_ARGS__);
#define vklLogW(fmt, ...) VklLogger::logW(fmt, ##__VA_ARGS__);

#define vklLogE(fmt, ...)                                                                \
  VklLogger::logE(LogUtil::extractFileName(__FILE__), __LINE__, fmt, ##__VA_ARGS__);