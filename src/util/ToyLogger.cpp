#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <filesystem>
#include "ToyLogger.h"
namespace toy {

std::shared_ptr<spdlog::logger> ToyLogger::logger;
std::mutex                      ToyLogger::loggerMutex;

void ToyLogger::init() {
  namespace fs = std::filesystem;
  fs::create_directories("log");

  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    "log/toylog.txt",
    true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};
  logger = std::make_shared<spdlog::logger>("TOY", sinks.begin(), sinks.end());
  logger->set_level(spdlog::level::debug);
}

void ToyLogger::ensureInit() {
  if (logger) {
    return;
  }

  std::lock_guard<std::mutex> lock(loggerMutex);
  if (!logger) {
    init();
  }
}

}  //namespace toy
