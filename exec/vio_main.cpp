#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"

io::Sensor*     sensor      = nullptr;
io::DataReader* dataReaders = nullptr;
std::string     dataPath    = "D:/dataset/EUROC/MH_01_easy";

void setupSensor() {
  sensor = io::SensorFactory::createSensor(io::SensorFactory::SensorType::SIMULATOR);

  io::DataReader* reader = io::DataReader::createDataReader(io::DataReader::Type::EUROC);
  reader->openDirectory();
  (io::Simulator*)sensor->registerDataReader(reader);

  sensor->prepare();
}

void registerCallbacks() {

  auto imageCallback = [](const int&      dataType,
                          const int&      format,
                          const uint64_t& ns,
                          uint8_t*        data,
                          const int&      length,
                          const int&      width,
                          const int&      height) {
    toy::SLAM::getInstance()
        ->setNewImage(dataType, format, ns, data, length, width, height);
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

std::string getConfigPath() {
  auto currPath   = std::filesystem::current_path();
  auto configPath = currPath.parent_path().parent_path().append("configs");
  return configPath.append("VioOnly.json").string();
}

int main() {
  setupSensor();

  registerCallbacks();

  auto configFile = getConfigPath();
  toy::SLAM::getInstance()->prepare(configFile);

  toy::SLAM::deleteInstance();

  delete reader;
  delete sensor;

  return 0;
}