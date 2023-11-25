#include "config.h"
#include "ToyLogger.h"
#include "ImagePyramid.h"
#include "MemoryPointerPool.h"
#include "Frame.h"
#include "Map.h"
#include "VioCore.h"
#include "Slam.h"
#include "types.h"

#include <opencv2/opencv.hpp>

namespace toy {

SLAM::SLAM()
  : mVioCore{nullptr} {};

SLAM::~SLAM() {
  delete mVioCore;
  mVioCore = nullptr;

  db::MemoryPointerPool::clear();
};

void SLAM::setSensorInfo(CameraInfo* cam0, CameraInfo* cam1, ImuInfo* imu) {
  Config::Vio::camInfo0 = *cam0;
  Config::Vio::camInfo1 = *cam1;

  if (imu) { Config::Vio::imuInfo = *imu; }
}

void SLAM::prepare(const std::string& configFile) {
  db::MemoryPointerPool::ready();
  Config::parseConfig(configFile);
  mVioCore = new VioCore();
  mVioCore->prepare();
}

void SLAM::setNewImage(ImageData& imageData0, ImageData& imageData1) {
  db::ImagePyramid* pyramids = new db::ImagePyramid[2]{imageData0, imageData1};
  mVioCore->insert(pyramids);

  if (Config::sync) mVioCore->processSync();
}

void SLAM::setAcc(const uint64_t& ns, float* acc) {}

void SLAM::setGyr(const uint64_t& ns, float* gyr) {}

}  //namespace toy
