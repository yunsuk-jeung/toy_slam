#include "config.h"
#include "ToyLogger.h"
#include "ImagePyramid.h"
#include "MemoryPointerPool.h"
#include "Frame.h"
#include "Map.h"
#include "Vio.h"
#include "Slam.h"
#include "types.h"

namespace toy {

SLAM::SLAM() : vio{nullptr} {};

SLAM::~SLAM() {
  delete vio;
  vio = nullptr;

  db::MemoryPointerPool::deleteInstance();
};

void SLAM::setSensorInfo(CameraInfo* cam0, CameraInfo* cam1, ImuInfo* imu) {
  memcpy(&Config::Vio::camInfo0, cam0, sizeof(CameraInfo));
  memcpy(&Config::Vio::camInfo1, cam1, sizeof(CameraInfo));

  if (imu) {
    memcpy(&Config::Vio::imuInfo, imu, sizeof(ImuInfo));
  }
}

void SLAM::prepare(const std::string& configFile) {
  db::MemoryPointerPool::getInstance();
  Config::parseConfig(configFile);
  vio = new Vio();
  vio->prepare();
}

void SLAM::setNewImage(ImageData& imageData0, ImageData& imageData1) {

  db::ImagePyramid* pyramids = new db::ImagePyramid[2]{imageData0, imageData1};
  vio->insert(pyramids);

  if (Config::sync) vio->process();
}

void SLAM::setAcc(const uint64_t& ns, float* acc) {}

void SLAM::setGyr(const uint64_t& ns, float* gyr) {}

}  //namespace toy
