#pragma once
#include <memory>
#include <iostream>
#include <string>
#include <iomanip>
#include <sophus/se3.hpp>
#include "Logger.h"


template <typename T, int Rows, int Cols>
struct fmt::formatter<Eigen::Matrix<T, Rows, Cols>> {
  static constexpr int eigenPrecision = 5;

  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }

  template <typename FormatContext>
  auto format(const Eigen::Matrix<T, Rows, Cols>& mat, FormatContext& ctx)
    -> decltype(ctx.out()) {
    static Eigen::IOFormat CleanMat(eigenPrecision,
                                    0,
                                    ", ",
                                    ",\n",
                                    "                                         [",
                                    "]",
                                    "-------------------------------------------\n",
                                    ",\n");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(eigenPrecision);
    ss << mat.format(CleanMat);

    return format_to(ctx.out(), "{}", ss.str());
  }
};

template <typename T, int Rows>
struct fmt::formatter<Eigen::Vector<T, Rows>> {
  static constexpr int eigenPrecision = 5;

  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }

  template <typename FormatContext>
  auto format(const Eigen::Vector<T, Rows>& vec, FormatContext& ctx)
    -> decltype(ctx.out()) {
    static Eigen::IOFormat CleanVec(eigenPrecision, 0, ", ", "\n", "[", "]");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(eigenPrecision);  //Set the precision
    ss << vec.transpose().format(CleanVec);

    return format_to(ctx.out(), "{}", ss.str());
  }
};

namespace toy {
class ToyLogger {
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
  template <typename Type, int Row>
  static std::string eigenVec(const Eigen::Matrix<Type, Row, 1>& vec, int precision = 4) {
    static Eigen::IOFormat CleanVec(precision, 0, ", ", "\n", "[", "]");
    CleanVec.precision = precision;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);  //Set the precision
    ss << vec.transpose().format(CleanVec);
    return ss.str();
  }

  template <typename Type, int Row, int Col>
  static std::string eigenMat(const Eigen::Matrix<Type, Row, Col>& mat,
                              int                                  precision = 4) {
    static Eigen::IOFormat CleanMat(precision, 0, ", ", "\n", "[", "]");
    CleanMat.precision = precision;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);  //Set the precision
    ss << "\n" << mat.format(CleanMat);
    return ss.str();
  }

private:
};
}  //namespace toy

#define ToyLogI(fmt, ...) ToyLogger::logI(fmt, ##__VA_ARGS__);
#define ToyLogW(fmt, ...) ToyLogger::logW(fmt, ##__VA_ARGS__);
#define ToyLogD(fmt, ...) ToyLogger::logD(fmt, ##__VA_ARGS__);
#define ToyLogE(fmt, ...)                                                                \
  ToyLogger::logE(LogUtil::extractFileName(__FILE__), __LINE__, fmt, ##__VA_ARGS__);
#define DEBUG_POINT() \
  ToyLogE("THIS Line is for debugging") ;
