#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"

io::Sensor*     sensor     = nullptr;
io::DataReader* dataReader = nullptr;
//std::string     dataPath         = "D:/dataset/EUROC/V1_01_easy";
//std::string dataPath         = "D:/dataset/EUROC/V1_03_difficult";
std::string dataPath         = "D:/dataset/EUROC/V1_01_easy";
std::string configPath       = "";
std::string sensorConfigFile = "euroc_sensor.json";
std::string slamConfigFile   = "VioOnly.json";

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
  sensor->start();
  std::cout << "SLAM is running. Press ENTER to stop." << std::endl;
  std::cin.get();
  sensor->stop();
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

#ifdef __linux__
  dataPath = dir.parent_path().append("EUROC").append("V1_01_easy").string();
#endif

  prepareSensor();
  prepareSLAM();
  runSLAM();

  toy::SLAM::deleteInstance();

  delete dataReader;
  delete sensor;

  return 0;
}
