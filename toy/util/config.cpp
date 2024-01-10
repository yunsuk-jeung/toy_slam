#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "ToyLogger.h"
#include "config.h"

namespace toy {
namespace {
int parseME(const std::string& name) {
  if (name == "huber")
    return 1;
  else
    return 0;
}
}  //namespace

bool Config::sync = {false};

CameraInfo Config::Vio::camInfo0;
CameraInfo Config::Vio::camInfo1;
ImuInfo    Config::Vio::imuInfo;

bool        Config::Vio::debug              = false;
int         Config::Vio::pyramidLevel       = 3;
int         Config::Vio::patchSize          = 52;
int         Config::Vio::rowGridCount       = 12;
int         Config::Vio::colGridCount       = 8;
std::string Config::Vio::pointTracker       = "Fast.OpticalflowLK";
int         Config::Vio::minTrackedPoint    = 30;
float       Config::Vio::minTrackedRatio    = 0.8;
double      Config::Vio::epipolarThreashold = 0.005;
bool        Config::Vio::showExtraction     = false;
bool        Config::Vio::showMonoTracking   = false;
bool        Config::Vio::showStereoTracking = false;

std::string Config::Vio::lineTracker           = "none";
bool        Config::Vio::frameTrackerSolvePose = false;

int    Config::Vio::initializeMapPointCount    = 30;
float  Config::Vio::newKeframeFeatureRatio     = 0.8;
float  Config::Vio::margFeatureConnectionRatio = 0.15;
int    Config::Vio::minKeyFrameCount           = 5;
int    Config::Vio::maxKeyFrameSize            = 7;
int    Config::Vio::maxFrameSize               = 3;
double Config::Vio::minParallaxSqNorm          = 20;

std::string Config::Vio::solverType            = "SqrtLocalSolver";
size_t      Config::Vio::solverMinimumFrames   = 4;
int         Config::Vio::reprojectionME        = 1;
double      Config::Vio::reprojectionMEConst   = 1.0;
double      Config::Vio::standardFocalLength   = 640.0;
int         Config::Vio::maxIteration          = 10;
bool        Config::Vio::compareLinearizedDiff = false;

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

  sync               = json["sync"];
  bool vio_on        = json["vio"]["on"];
  Config::Vio::debug = json["vio"]["debug"];

  auto frameTrackerJson = json["vio"]["frameTracker"];
  Vio::pyramidLevel     = frameTrackerJson["pyramidLevel"];

  auto feautreJson = frameTrackerJson["feature"];

  auto pointJson          = feautreJson["point"];
  bool point_on           = pointJson["on"];
  Vio::patchSize          = pointJson["patchSize"];
  Vio::rowGridCount       = pointJson["rowGridCount"];
  Vio::colGridCount       = pointJson["colGridCount"];
  Vio::pointTracker       = pointJson["tracker"];
  Vio::minTrackedPoint    = pointJson["minTrackedPoint"];
  Vio::minTrackedRatio    = pointJson["minTrackedRatio"];
  Vio::epipolarThreashold = pointJson["epipolarThreashold"];
  Vio::showExtraction     = pointJson["showExtraction"];
  Vio::showMonoTracking   = pointJson["showMonoTracking"];
  Vio::showStereoTracking = pointJson["showStereoTracking"];

  bool line_on               = feautreJson["line"]["on"];
  Vio::frameTrackerSolvePose = frameTrackerJson["solvePose"];

  auto localTrackerJson           = json["vio"]["localTracker"];
  Vio::initializeMapPointCount    = localTrackerJson["initializeMapPointCount"];
  Vio::newKeframeFeatureRatio     = localTrackerJson["newKeframeFeatureRatio"];
  Vio::margFeatureConnectionRatio = localTrackerJson["margFeatureConnectionRatio"];
  Vio::minKeyFrameCount           = localTrackerJson["minKeyFrameCount"];
  Vio::maxFrameSize               = localTrackerJson["maxFrameSize"];
  Vio::maxKeyFrameSize            = localTrackerJson["maxKeyFrameSize"];
  Vio::minParallaxSqNorm          = localTrackerJson["minParallaxSqNorm"];

  auto vioSolverJson = localTrackerJson["vioSolver"];
  Vio::solverType    = vioSolverJson["name"];
  {
    std::string me      = vioSolverJson["reprojectionME"];
    Vio::reprojectionME = parseME(me);
  }
  Vio::solverMinimumFrames   = vioSolverJson["solverMinimumFrames"];
  Vio::reprojectionMEConst   = vioSolverJson["reprojectionMEConst"];
  Vio::standardFocalLength   = vioSolverJson["standardFocalLength"];
  Vio::maxIteration          = vioSolverJson["maxIteration"];
  Vio::compareLinearizedDiff = vioSolverJson["compareLinearizedDiff"];

  auto basicSolverJson  = json["basicSolver"];
  Solver::basicMinDepth = basicSolverJson["minDepth"];
  Solver::basicMaxDepth = basicSolverJson["maxDepth"];

  int align_width = 10;
  ToyLogI("sync mode : {}", sync);
  ToyLogI("################  vio  ################");
  if (vio_on) {
    ToyLogI("----------- tracker point ----------");
    ToyLogI("- {:<{}} :   {}", "On", align_width, point_on);
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
