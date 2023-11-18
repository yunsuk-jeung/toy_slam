#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "ToyLogger.h"
#include "config.h"

namespace toy {
bool Config::sync = {false};

CameraInfo Config::Vio::camInfo0;
CameraInfo Config::Vio::camInfo1;
ImuInfo    Config::Vio::imuInfo;

std::string Config::Vio::pointExtractor = "FAST";

std::string Config::Vio::pointMatcher = "OpticalFlowCV";
int         Config::Vio::pyramidLevel = 3;
int         Config::Vio::patchSize    = 52;

std::string Config::Vio::lineExtractor = "none";
std::string Config::Vio::lineMatcher   = "none";
std::string Config::Vio::solverType    = "SqrtLocalSolver";

void Config::parseConfig(const std::string& configFile) {
  spdlog::set_level(spdlog::level::debug);

  std::ifstream file(configFile);
  if (!file.is_open()) {
    ToyLogE("config file path is wrong : {}", configFile);
    throw std::runtime_error("config File is wrong!");
  }

  nlohmann::json json_obj;
  file >> json_obj;

  file.close();

  sync                = json_obj["Sync"];
  bool vio_on         = json_obj["Vio"]["On"];
  bool point_on       = json_obj["Vio"]["Tracker"]["Point"]["On"];
  Vio::pointExtractor = json_obj["Vio"]["Tracker"]["Point"]["Extractor"];

  auto matcherObj   = json_obj["Vio"]["Tracker"]["Point"]["Matcher"];
  Vio::pointMatcher = matcherObj["Type"];
  Vio::pyramidLevel = matcherObj["PyramidLevel"];
  Vio::patchSize    = matcherObj["PatchSize"];

  bool line_on    = json_obj["Vio"]["Tracker"]["Line"]["On"];
  Vio::solverType = json_obj["Vio"]["Solver"];

  int align_width = 10;
  ToyLogI("sync mode : {}", sync);
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
