#include <iostream>
#include <functional>
#include <filesystem>

#include "DataReader.h"
#include "Simulator.h"
#include "SensorFactory.h"
#include "SLAM.h"
#include "SLAMApp.h"
#include "GlfwWindow.h"

vkl::SLAMApp*   app        = nullptr;
io::Sensor*     sensor     = nullptr;
io::DataReader* dataReader = nullptr;
//std::string     dataPath         = "D:/dataset/EUROC/V1_01_easy";
//std::string dataPath         = "D:/dataset/EUROC/V1_03_difficult";
std::string dataPath         = "D:/dataset/EUROC/V1_01_easy";
std::string configPath       = "";
std::string sensorConfigFile = "euroc_sensor.json";
std::string slamConfigFile   = "VioOnly.json";
std::string shaderPath       = "";

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

void prepareGUI() {
  vkl::WindowInfo winInfo{
    "Sample window",
    vkl::WindowInfo::Mode::Default,
    true,
    vkl::WindowInfo::Vsync::ON,
    vkl::WindowInfo::Orientation::Landscape,
    {1280, 1280}
  };

  app = new vkl::SLAMApp();
  std::unique_ptr<vkl::Window> glfwWindow(new vkl::GlfwWindow(winInfo, app));
  app->registerWindow(glfwWindow);
  app->addShaderPath(shaderPath);
  app->addAssetPath(shaderPath);
  app->registerSensor(sensor);
  app->prepare();
}

int main() {
  namespace fs = std::filesystem;
  fs::path currFile(std::string(__FILE__));
  auto     dir       = currFile.parent_path().parent_path();
  auto     shaderDir = dir;
  auto     configDir = dir;

  std::cout << dir.string() << std::endl;

  shaderPath       = shaderDir.append("libvulkanlight/shaders").string();
  configPath       = configDir.append("configs").string();
  sensorConfigFile = configPath + "/" + sensorConfigFile;
  slamConfigFile   = configPath + "/" + slamConfigFile;

#ifdef __linux__
  dataPath = dir.parent_path().append("V1_01_easy");
#endif

  prepareSensor();
  prepareSLAM();
  prepareGUI();

  app->run();

  toy::SLAM::deleteInstance();

  delete dataReader;
  delete sensor;
  delete app;

  return 0;
}