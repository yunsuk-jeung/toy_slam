#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "Logger.h"
#include "config.h"

namespace toy {
std::string Config::Vio::pointExtractor = "FAST";
std::string Config::Vio::pointMatcher   = "OpticalFlowCV";
std::string Config::Vio::lineExtractor  = "none";
std::string Config::Vio::lineMatcher    = "none";
std::string Config::Vio::solverType     = "SqrtLocalSolver";

void Config::parseConfig(const std::string& configFile) {
  spdlog::set_level(spdlog::level::debug);

  std::ifstream file(configFile);

  //파일에서 JSON 데이터 읽기
  nlohmann::json json_obj;
  file >> json_obj;

  //파일 닫기
  file.close();

  bool vio_on         = json_obj["Vio"]["On"];
  bool point_on       = json_obj["Vio"]["Tracker"]["Point"]["On"];
  Vio::pointExtractor = json_obj["Vio"]["Tracker"]["Point"]["Extractor"];
  Vio::pointMatcher   = json_obj["Vio"]["Tracker"]["Point"]["Matcher"];
  bool line_on        = json_obj["Vio"]["Tracker"]["Line"]["On"];
  Vio::solverType     = json_obj["Vio"]["Solver"];

  int align_width = 10;
  LOGI("################  vio  ################");
  //LOGI("{:<{}} -   {}", "On", align_width, vio_on);
  if (vio_on) {
    LOGI("----------- tracker point ----------");
    LOGI("-{:<{}} -   {}", "On", align_width, point_on);
    LOGI("-{:<{}} -   {}", "Extractor", align_width, Vio::pointExtractor);
    LOGI("-{:<{}} -   {}", "Matcher", align_width, Vio::pointMatcher);
    LOGI("----------- tracker line ----------")
    LOGI("-{:<{}} -   {}", "On", align_width, line_on);
    LOGI("----------- solver ----------");
    LOGI("-{:<{}} -   {}", "Solver", align_width, Vio::solverType);
    LOGI("################  end  ################");
  }
}

}  //namespace toy
