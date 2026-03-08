#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "ToyLogger.h"
namespace toy {

std::shared_ptr<spdlog::logger> ToyLogger::logger;

void ToyLogger::init() {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("log/toylog.txt",
                                                                       true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};
  logger = std::make_shared<spdlog::logger>("TOY", sinks.begin(), sinks.end());
  logger->set_level(spdlog::level::debug);
}

}  //namespace toy