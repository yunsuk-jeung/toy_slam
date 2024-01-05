#include <memory>
#include <iostream>
#include <string>
#include <iomanip>
#include <sophus/se3.hpp>
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/fmt/ostr.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include "Logger.h"

std::shared_ptr<spdlog::logger> logger;

enum class color { red, green, blue };

template <>
struct fmt::formatter<color> : formatter<string_view> {
  //parse is inherited from formatter<string_view>.

  auto format(color c, format_context& ctx) const;
};

auto fmt::formatter<color>::format(color c, format_context& ctx) const {
  string_view name = "unknown";
  switch (c) {
  case color::red:
    name = "red";
    break;
  case color::green:
    name = "green";
    break;
  case color::blue:
    name = "blue";
    break;
  }
  return formatter<string_view>::format(name, ctx);
}

template <typename T, int Rows, int Cols>
struct fmt::formatter<Eigen::Matrix<T, Rows, Cols>> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }

  template <typename FormatContext>
  auto format(const Eigen::Matrix<T, Rows, Cols>& mat, FormatContext& ctx)
    -> decltype(ctx.out()) {
    static Eigen::IOFormat CleanMat(5, 0, ", ", "\n", "[", "]");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(5);
    ss << "\n" << mat.format(CleanMat);

    return format_to(ctx.out(), "{}", ss.str());
  }
};

template <typename T, int Rows>
struct fmt::formatter<Eigen::Vector<T, Rows>> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.end();
  }

  template <typename FormatContext>
  auto format(const Eigen::Vector<T, Rows>& vec, FormatContext& ctx)
    -> decltype(ctx.out()) {
    static Eigen::IOFormat CleanVec(5, 0, ", ", "\n", "[", "]");

    std::stringstream ss;
    ss << std::fixed << std::setprecision(5);  //Set the precision
    ss << vec.transpose().format(CleanVec);

    return format_to(ctx.out(), "{}", ss.str());
  }
};

auto main() -> int {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("log/toylog.txt",
                                                                       true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  std::vector<spdlog::sink_ptr>   sinks{file_sink, console_sink};
  std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>("sample",
                                                                            sinks.begin(),
                                                                            sinks.end());
  Eigen::Vector3d                 A(1, 0, 0);

  logger->info("test {}", A);

  return 0;
}