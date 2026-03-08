#include <iostream>
#include <functional>
#include <filesystem>
#include <atomic>
#include <thread>
#include <chrono>

#include <rerun.hpp>
#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"
#include "SLAMInfo.h"

io::Sensor*     sensor     = nullptr;
io::DataReader* dataReader = nullptr;
std::string     dataPath = "/Users/yunsuk/workspace/omni_slam/datasets/EUROC/V1_01_easy";
std::string     configPath       = "";
std::string     sensorConfigFile = "euroc_sensor.json";
std::string     slamConfigFile   = "VioOnly.json";

namespace {

constexpr float kAxisLength = 0.08f;

rerun::Vec3D toTranslation(const Eigen::Matrix4f& Mwc) {
  const Eigen::Vector3f t = Mwc.block<3, 1>(0, 3);
  return {t.x(), t.y(), t.z()};
}

void logLatestAccumulatedPath(const rerun::RecordingStream&     rec,
                              const std::vector<rerun::Vec3D>& accumulatedPath) {
  if (accumulatedPath.empty()) {
    return;
  }

  std::vector<rerun::components::LineStrip3D> latestPathStrip;
  latestPathStrip.emplace_back(accumulatedPath);
  rec.log("slam/latest_path", rerun::LineStrips3D(latestPathStrip));
}

void logPathAndAxes(const rerun::RecordingStream& rec, const std::vector<Eigen::Matrix4f>& paths) {
  if (paths.empty()) {
    return;
  }

  std::vector<rerun::Position3D> axisOrigins;
  std::vector<rerun::Vector3D>   axisVectors;
  std::vector<rerun::Color>      axisColors;
  axisOrigins.reserve(paths.size() * 3);
  axisVectors.reserve(paths.size() * 3);
  axisColors.reserve(paths.size() * 3);

  for (const auto& Mwc : paths) {
    const Eigen::Vector3f t = Mwc.block<3, 1>(0, 3);
    const Eigen::Matrix3f R = Mwc.block<3, 3>(0, 0);

    axisOrigins.emplace_back(t.x(), t.y(), t.z());
    axisVectors.emplace_back(R(0, 0) * kAxisLength, R(1, 0) * kAxisLength, R(2, 0) * kAxisLength);
    axisColors.emplace_back(255, 64, 64);

    axisOrigins.emplace_back(t.x(), t.y(), t.z());
    axisVectors.emplace_back(R(0, 1) * kAxisLength, R(1, 1) * kAxisLength, R(2, 1) * kAxisLength);
    axisColors.emplace_back(64, 255, 64);

    axisOrigins.emplace_back(t.x(), t.y(), t.z());
    axisVectors.emplace_back(R(0, 2) * kAxisLength, R(1, 2) * kAxisLength, R(2, 2) * kAxisLength);
    axisColors.emplace_back(64, 64, 255);
  }

  rec.log("slam/frame_axes",
          rerun::Arrows3D::from_vectors(axisVectors).with_origins(axisOrigins).with_colors(
            axisColors));
}

void logMapPoints(const rerun::RecordingStream& rec, const std::vector<float>& localPoints) {
  std::vector<rerun::Position3D> mapPoints;
  mapPoints.reserve(localPoints.size() / 4);

  for (size_t i = 0; i + 3 < localPoints.size(); i += 4) {
    mapPoints.emplace_back(localPoints[i], localPoints[i + 1], localPoints[i + 2]);
  }

  if (!mapPoints.empty()) {
    rec.log("slam/map_points", rerun::Points3D(mapPoints));
  }
}

void rerunLoop(std::atomic<bool>& running) {
  rerun::RecordingStream rec("toy_slam");
  rec.spawn().exit_on_failure();

  auto*                     info = toy::SLAMInfo::getInstance();
  int64_t                   frameNr = 0;
  std::vector<rerun::Vec3D> accumulatedPath;

  while (running.load()) {
    bool updated = false;

    std::vector<Eigen::Matrix4f> localPath;
    if (info->getLocalPath(localPath)) {
      rec.set_time_sequence("frame", frameNr);
      logPathAndAxes(rec, localPath);
      if (!localPath.empty()) {
        accumulatedPath.push_back(toTranslation(localPath.back()));
        logLatestAccumulatedPath(rec, accumulatedPath);
      }
      updated = true;
    }

    std::vector<float> localPoints;
    if (info->getLocalPoints(localPoints)) {
      if (!updated) {
        rec.set_time_sequence("frame", frameNr);
      }
      logMapPoints(rec, localPoints);
      updated = true;
    }

    if (updated) {
      ++frameNr;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

void setupSensor() {
  sensor = io::SensorFactory::createSensor(io::SensorFactory::SensorType::SIMULATOR);

  if (sensor->isSimulator()) {
    dataReader = io::DataReader::createDataReader(io::DataReader::Type::EUROC);
    dataReader->openDirectory(sensorConfigFile, dataPath);
    ((io::Simulator*)sensor)->registerDataReader(dataReader);
  }
  sensor->prepare();
}

void registerCallbacks() {
  auto imageCallback = [](std::vector<ImageData>& s) {
    toy::SLAM::getInstance()->setNewImages(s);
  };

  auto accCallback = [](const uint64_t& ns, float* acc) {
    toy::SLAM::getInstance()->setAcc(ns, acc);
  };

  auto gyrCallback = [](const uint64_t& ns, float* gyr) {
    toy::SLAM::getInstance()->setGyr(ns, gyr);
  };

  sensor->registerImageCallback(imageCallback);
  sensor->registerAccCallback(accCallback);
  sensor->registerGyrCallback(gyrCallback);
}

void prepareSensor() {
  setupSensor();
  registerCallbacks();
}

void prepareSLAM() {
  CameraInfo info0;
  CameraInfo info1;
  sensor->getInfo(&info0, &info1);

  toy::SLAM::getInstance()->setSensorInfo(&info0, &info1);
  toy::SLAM::getInstance()->prepare(slamConfigFile);
}

void runSLAM() {
  std::atomic<bool> rerunRunning{true};
  std::thread       rerunThread(rerunLoop, std::ref(rerunRunning));

  sensor->start();
  std::cout << "SLAM is running. Waiting for dataset end..." << std::endl;

  if (sensor->isSimulator()) {
    auto* simulator = static_cast<io::Simulator*>(sensor);
    while (simulator->isWorking()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  sensor->stop();

  rerunRunning = false;
  if (rerunThread.joinable()) {
    rerunThread.join();
  }
}

int main() {
  namespace fs = std::filesystem;
  fs::path currFile(std::string(__FILE__));
  auto     dir       = currFile.parent_path().parent_path();
  auto     configDir = dir;

  std::cout << dir.string() << std::endl;

  configPath       = configDir.append("configs").string();
  sensorConfigFile = configPath + "/" + sensorConfigFile;
  slamConfigFile   = configPath + "/" + slamConfigFile;

  prepareSensor();
  prepareSLAM();
  runSLAM();

  toy::SLAM::deleteInstance();

  delete dataReader;
  delete sensor;

  return 0;
}
