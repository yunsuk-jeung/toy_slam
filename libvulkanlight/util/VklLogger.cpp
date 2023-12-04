#include "spdlog/sinks/basic_file_sink.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include "VklLogger.h"

namespace vkl {
std::shared_ptr<spdlog::logger> VklLogger::logger;
void VklLogger::init() {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("log/vkllog.txt",
                                                                       true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};
  logger = std::make_shared<spdlog::logger>("VKL", sinks.begin(), sinks.end());
  logger->set_level(spdlog::level::debug);
}

}  //namespace vkl