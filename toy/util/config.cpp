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

int         Config::Vio::pyramidLevel       = 3;
int         Config::Vio::patchSize          = 52;
int         Config::Vio::rowGridCount       = 12;
int         Config::Vio::colGridCount       = 8;
std::string Config::Vio::pointTracker       = "Fast.OpticalflowLK";
double      Config::Vio::epipolarThreashold = 0.005;

std::string Config::Vio::lineTracker           = "none";
bool        Config::Vio::frameTrackerSolvePose = false;

std::string Config::Vio::solverType              = "SqrtLocalSolver";
int         Config::Vio::initializeMapPointCount = 30;

int Config::Vio::mapFrameSize = 6;

double Config::Solver::basicMinDepth = 0.005;
double Config::Solver::basicMaxDepth = 140;

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

  sync        = json["sync"];
  bool vio_on = json["vio"]["on"];

  auto frameTrackerJson = json["vio"]["frameTracker"];
  Vio::pyramidLevel     = frameTrackerJson["pyramidLevel"];

  auto feautreJson        = frameTrackerJson["feature"];
  bool point_on           = feautreJson["point"]["on"];
  Vio::patchSize          = feautreJson["point"]["patchSize"];
  Vio::rowGridCount       = feautreJson["point"]["rowGridCount"];
  Vio::colGridCount       = feautreJson["point"]["colGridCount"];
  Vio::pointTracker       = feautreJson["point"]["tracker"];
  Vio::epipolarThreashold = feautreJson["point"]["epipolarThreashold"];

  bool line_on               = feautreJson["line"]["on"];
  Vio::frameTrackerSolvePose = frameTrackerJson["solvePose"];

  auto localTrackerJson        = json["vio"]["localTracker"];
  Vio::solverType              = localTrackerJson["solver"];
  Vio::initializeMapPointCount = localTrackerJson["initializeMapPointCount"];

  Vio::mapFrameSize = json["vio"]["localMap"]["frameSize"];

  auto solverJson       = json["solver"];
  Solver::basicMinDepth = solverJson["basic"]["minDepth"];
  Solver::basicMaxDepth = solverJson["basic"]["maxDepth"];

  int align_width = 10;
  ToyLogI("sync mode : {}", sync);
  ToyLogI("################  vio  ################");
  if (vio_on) {
    ToyLogI("----------- tracker point ----------");
    ToyLogI("- k{:<{}} :   {}", "On", align_width, point_on);
    ToyLogI("- {:<{}} :   {}", "Tracker", align_width, Vio::pointTracker);
    ToyLogI("");

    ToyLogI("----------- tracker line ----------");
    ToyLogI("- {:<{}} :   {}", "On", align_width, line_on);
    ToyLogI("");

    ToyLogI("----------- solver ----------");
    ToyLogI("- {:<{}} :   {}", "Solver", align_width, Vio::solverType);
  }
  ToyLogI("################  solver  ################");
  ToyLogI("- {:<{}} :   {}", "basic min depth", align_width, Solver::basicMinDepth);
  ToyLogI("- {:<{}} :   {}", "basic max depth", align_width, Solver::basicMaxDepth);
  ToyLogI("################  end  ################");
}

}  //namespace toy
