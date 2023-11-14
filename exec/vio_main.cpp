#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"

std::string getConfigPath() {
  auto currPath   = std::filesystem::current_path();
  auto configPath = currPath.parent_path().parent_path().append("configs");
  return configPath.append("VioOnly.json").string();
}

void registerCallbacks(io::Sensor* sensor) {

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

int main() {
  auto* sensor = (io::Simulator*)io::SensorFactory::createSensor(
      io::SensorFactory::SensorType::SIMULATOR);

  io::DataReader* reader = io::DataReader::createDataReader(io::DataReader::Type::EUROC);

  sensor->registerDataReader(reader);
  sensor->prepare();

  registerCallbacks(sensor);

  auto configFile = getConfigPath();
  toy::SLAM::getInstance()->prepare(configFile);

  toy::SLAM::deleteInstance();

  delete reader;
  delete sensor;

  return 0;
}