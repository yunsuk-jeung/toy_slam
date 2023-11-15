#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "Slam.h"

io::Sensor*     sensor           = nullptr;
io::DataReader* dataReader       = nullptr;
std::string     dataPath         = "D:/dataset/EUROC/MH_04_difficult";
std::string     sensorConfigFile = "F:/transfer/toy_slam/configs/euroc_sensor.json";
std::string     slamConfigFile   = "F:/transfer/toy_slam/configs/VioOnly.json";

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

  auto imageCallback = [](const int&      dataType,
                          const int&      format,
                          const uint64_t& ns,
                          uint8_t*        data,
                          const int&      width,
                          const int&      height) {
    toy::SLAM::getInstance()->setNewImage(dataType, format, ns, data, width, height);
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

  float info0[28];
  float info1[28];
  sensor->getInfo(info0, info1);

  toy::SLAM::getInstance()->setSensorInfo(info0, info1);
  toy::SLAM::getInstance()->prepare(slamConfigFile);

  sensor->start();

  cv::Mat tempGUi(1000, 1000, CV_8UC1);
  cv::imshow("test", tempGUi);
  int key = cv::waitKey();

  sensor->stop();

  toy::SLAM::deleteInstance();

  delete dataReader;
  delete sensor;

  return 0;
}