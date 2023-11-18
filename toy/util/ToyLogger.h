#pragma once
#include <iostream>
#include <string>
#include <iomanip>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <sophus/se3.hpp>

#ifdef __ANDROID__
#include <spdlog/sinks/android_sink.h>
#define ROOT_PATH_SIZE 36
#else
#define ROOT_PATH_SIZE 60
#endif

#define LOGGER_FORMAT "[%^%l%$] %v"
#define PROJECT_NAME "TOY"

#define __FILENAME__ LogUtil::extractFileName(__FILE__)
#ifdef _WIN32

namespace toy {

class LogUtil {
public:
  static std::string SE3String(const Sophus::SE3d& se3, int precision = 4) {
    Eigen::IOFormat CleanMat(precision, 0, ", ", "\n", "             [", "]", "\n");
    Eigen::IOFormat CleanVec(precision, 0, ", ", "\n", "[", "]");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);  //Set the precision
    ss << "\nRotation:" << se3.rotationMatrix().format(CleanMat) << "\n"
       << "Translation: \n"
       << se3.translation().transpose().format(CleanVec);
    return ss.str();
  }
  static std::string se3String(const Sophus::SE3d& se3, int precision = 4) {
    Eigen::IOFormat CleanVec(precision, 0, ", ", "\n", "[", "]");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);  //Set the precision
    ss << "\n translation_so3 : " << se3.log().transpose().format(CleanVec);
    return ss.str();
  }

  static const char* extractFileName(const char* filePath) {
#ifdef _WIN32
    const char* lastSlash = strrchr(filePath, '\\');
#else
    const char* lastSlash = strrchr(filePath, '/');
#endif
    if (lastSlash != nullptr) {
      return lastSlash + 1;
    }
    else {
      return filePath;
    }
  }
};
}  //namespace toy

#define ToyLogI(...) spdlog::info(__VA_ARGS__);
#define ToyLogW(...) spdlog::warn(__VA_ARGS__);
#define ToyLogE(...)                                                                     \
  spdlog::error("[{}:{}] {}", __FILENAME__, __LINE__, fmt::format(__VA_ARGS__));
#define ToyLogD(...) spdlog::debug(__VA_ARGS__);

#elif __ANDROID__
class ToyLogger {
  static std::shared_ptr<spdlog::logger> android_logger;

public:
  template <typename... Args>
  static void logI(const char* fmt, Args... args) {
    android_logger->info(fmt, args...);
  }
  template <typename... Args>
  static void logW(const char* fmt, Args... args) {
    android_logger->warn(fmt, args...);
  }
  template <typename... Args>
  static void logD(const char* fmt, Args... args) {
    android_logger->debug(fmt, args...);
  }

  template <typename... Args>
  static void logE(const char* fmt, Args... args) {
    android_logger->error("[{}:{}] {}",
                          __FILENAME__,
                          __LINE__,
                          fmt::format(fmt, args...));
  }
};

#define ToyLogI(fmt, ...) ToyLogger::logI(fmt, ##__VA_ARGS__);
#define ToyLogW(fmt, ...) ToyLogger::logW(fmt, ##__VA_ARGS__);
#define ToyLogE(fmt, ...) ToyLogger::logE(fmt, ##__VA_ARGS__);
#define ToyLogD(fmt, ...) ToyLogger::logD(fmt, ##__VA_ARGS__);

#endif