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

int         Config::Vio::pyramidLevel          = 3;
int         Config::Vio::patchSize             = 52;
int         Config::Vio::rowGridCount          = 12;
int         Config::Vio::colGridCount          = 8;
std::string Config::Vio::pointTracker          = "Fast.OpticalflowLK";
std::string Config::Vio::lineTracker           = "none";
bool        Config::Vio::frameTrackerSolvePose = false;

std::string Config::Vio::solverType = "SqrtLocalSolver";

int Config::Vio::mapFrameSize = 6;

void Config::parseConfig(const std::string& configFile) {
  spdlog::set_level(spdlog::level::debug);

  std::ifstream file(configFile);
  if (!file.is_open()) {
    ToyLogE("config file path is wrong : {}", configFile);
    throw std::runtime_error("config File is wrong!");
  }

  nlohmann::json json;
  file >> json;

  file.close();

  sync        = json["Sync"];
  bool vio_on = json["Vio"]["On"];

  auto frameTrackerJson = json["Vio"]["FrameTracker"];
  Vio::pyramidLevel     = frameTrackerJson["PyramidLevel"];

  auto feautreJson  = frameTrackerJson["Feature"];
  bool point_on     = feautreJson["Point"]["On"];
  Vio::patchSize    = feautreJson["Point"]["PatchSize"];
  Vio::rowGridCount = feautreJson["Point"]["RowGridCount"];
  Vio::colGridCount = feautreJson["Point"]["ColGridCount"];
  Vio::pointTracker = feautreJson["Point"]["Tracker"];

  bool line_on               = feautreJson["Line"]["On"];
  Vio::frameTrackerSolvePose = frameTrackerJson["SolvePose"];

  Vio::solverType   = json["Vio"]["Solver"];
  Vio::mapFrameSize = json["Vio"]["LocalMap"]["FrameSize"];

  int align_width = 10;
  ToyLogI("sync mode : {}", sync);
  ToyLogI("################  vio  ################");
  if (vio_on) {
    ToyLogI("----------- tracker point ----------");
    ToyLogI("-{:<{}} -   {}", "On", align_width, point_on);
    ToyLogI("-{:<{}} -   {}", "Tracker", align_width, Vio::pointTracker);
    ToyLogI("");

    ToyLogI("----------- tracker line ----------");
    ToyLogI("-{:<{}} -   {}", "On", align_width, line_on);
    ToyLogI("");

    ToyLogI("----------- solver ----------");
    ToyLogI("-{:<{}} -   {}", "Solver", align_width, Vio::solverType);
    ToyLogI("################  end  ################");
  }
}

}  //namespace toy
