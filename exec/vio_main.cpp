#include <iostream>
#include <functional>
#include <filesystem>

#include "Sensor.h"
#include "SensorFactory.h"
#include "Slam.h"

std::string getConfigPath() {
  auto currPath = std::filesystem::current_path();
  auto configPath =
      currPath.parent_path().parent_path().append("configs").append("VioOnly.json");
  return configPath.string();
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
}

int main() {
  std::unique_ptr<io::Sensor> sensorUptr =
      io::SensorFactory::createSensor(io::SensorFactory::SensorType::SIMULATOR);

  registerCallbacks(sensorUptr.get());

  auto configFile = getConfigPath();
  toy::SLAM::getInstance()->prepare(configFile);

  toy::SLAM::deleteInstance();
  return 0;
}