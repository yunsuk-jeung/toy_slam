#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"

io::Sensor*     sensor           = nullptr;
io::DataReader* dataReader       = nullptr;
std::string     dataPath         = "D:/dataset/EUROC/MH_01_easy";
std::string     sensorConfigFile = "D:/workspaceD/toy_vio/configs/euroc_sensor.json";
std::string     slamConfigFile   = "D:/workspaceD/toy_vio/configs/VioOnly.json";

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

  auto imageCallback = [](ImageData& imageData0, ImageData& imageData1) {
    toy::SLAM::getInstance()->setNewImage(imageData0, imageData1);
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
  setupSensor();
  registerCallbacks();

  CameraInfo info0;
  CameraInfo info1;
  sensor->getInfo(&info0, &info1);

  toy::SLAM::getInstance()->setSensorInfo(&info0, &info1);
  toy::SLAM::getInstance()->prepare(slamConfigFile);

  //sensor->start();
  while (true) {
    ((io::Simulator*)sensor)->spinOnce();
    int key = cv::waitKey();
    if (key == 27) break;
  }

  sensor->stop();

  toy::SLAM::deleteInstance();

  delete dataReader;
  delete sensor;

  return 0;
}