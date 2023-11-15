#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "ToyLogger.h"
#include "config.h"

namespace toy {
bool Config::sync = {false};

float Config::Vio::camInfo0[12] = {0};
float Config::Vio::camInfo1[12] = {0};
float Config::Vio::imuInfo[4]   = {0};

std::string Config::Vio::pointExtractor = "FAST";
std::string Config::Vio::pointMatcher   = "OpticalFlowCV";
std::string Config::Vio::lineExtractor  = "none";
std::string Config::Vio::lineMatcher    = "none";
std::string Config::Vio::solverType     = "SqrtLocalSolver";

void Config::parseConfig(const std::string& configFile) {
  spdlog::set_level(spdlog::level::debug);

  std::ifstream file(configFile);

  nlohmann::json json_obj;
  file >> json_obj;

  file.close();

  sync                = json_obj["Sync"];
  bool vio_on         = json_obj["Vio"]["On"];
  bool point_on       = json_obj["Vio"]["Tracker"]["Point"]["On"];
  Vio::pointExtractor = json_obj["Vio"]["Tracker"]["Point"]["Extractor"];
  Vio::pointMatcher   = json_obj["Vio"]["Tracker"]["Point"]["Matcher"];
  bool line_on        = json_obj["Vio"]["Tracker"]["Line"]["On"];
  Vio::solverType     = json_obj["Vio"]["Solver"];

  int align_width = 10;
  ToyLogI("sync mode : ", sync);
  ToyLogI("################  vio  ################");
  if (vio_on) {
    ToyLogI("----------- tracker point ----------");
    ToyLogI("-{:<{}} -   {}", "On", align_width, point_on);
    ToyLogI("-{:<{}} -   {}", "Extractor", align_width, Vio::pointExtractor);
    ToyLogI("-{:<{}} -   {}", "Matcher", align_width, Vio::pointMatcher);
    ToyLogI("----------- tracker line ----------")
        ToyLogI("-{:<{}} -   {}", "On", align_width, line_on);
    ToyLogI("----------- solver ----------");
    ToyLogI("-{:<{}} -   {}", "Solver", align_width, Vio::solverType);
    ToyLogI("################  end  ################");
  }
}

}  //namespace toy
