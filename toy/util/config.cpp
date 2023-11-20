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

int         Config::Vio::pyramidLevel = 3;
int         Config::Vio::patchSize    = 52;
int         Config::Vio::rowGridCount = 12;
int         Config::Vio::colGridCount = 8;
std::string Config::Vio::pointTracker = "Fast.OpticalflowLK";
std::string Config::Vio::lineTracker  = "none";

std::string Config::Vio::solverType = "SqrtLocalSolver";

int Config::Vio::mapFrameSize = 6;

void Config::parseConfig(const std::string& configFile) {
  spdlog::set_level(spdlog::level::debug);

  std::ifstream file(configFile);
  if (!file.is_open()) {
    ToyLogE("config file path is wrong : {}", configFile);
    throw std::runtime_error("config File is wrong!");
  }

  nlohmann::json jsonObj;
  file >> jsonObj;

  file.close();

  sync        = jsonObj["Sync"];
  bool vio_on = jsonObj["Vio"]["On"];

  auto trackerObj = jsonObj["Vio"]["Tracker"];

  Vio::pyramidLevel = trackerObj["PyramidLevel"];
  bool point_on     = trackerObj["Point"]["On"];
  Vio::patchSize    = trackerObj["Point"]["PatchSize"];
  Vio::rowGridCount = trackerObj["Point"]["RowGridCount"];
  Vio::colGridCount = trackerObj["Point"]["ColGridCount"];
  Vio::pointTracker = trackerObj["Point"]["Tracker"];

  bool line_on = trackerObj["Line"]["On"];

  Vio::solverType = jsonObj["Vio"]["Solver"];

  Vio::mapFrameSize = jsonObj["Vio"]["LocalMap"]["FrameSize"];

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
