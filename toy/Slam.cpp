#include "config.h"
#include "ToyLogger.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Map.h"
#include "VioCore.h"
#include "Slam.h"
#include "types.h"

#include <opencv2/opencv.hpp>

namespace toy {

SLAM::SLAM()
  : mVioCore{nullptr} {
  ToyLogger::init();
};

SLAM::~SLAM() {
  delete mVioCore;
  mVioCore = nullptr;
};

void SLAM::setSensorInfo(CameraInfo* cam0, CameraInfo* cam1, ImuInfo* imu) {
  Config::Vio::camInfo0 = *cam0;
  Config::Vio::camInfo1 = *cam1;

  if (imu) {
    Config::Vio::imuInfo = *imu;
  }
}

void SLAM::prepare(const std::string& configFile) {
  Config::parseConfig(configFile);
  mVioCore = new VioCore();
  mVioCore->prepare();
}

void SLAM::setNewImage(ImageData& imageData0, ImageData& imageData1) {
  db::ImagePyramid* imagePyramid0{new db::ImagePyramid(imageData0)};
  db::ImagePyramid* imagePyramid1{new db::ImagePyramid(imageData1)};

  db::ImagePyramidSet::Ptr set = std::make_shared<db::ImagePyramidSet>(imagePyramid0,
                                                                       imagePyramid1);
  mVioCore->insert(set);

  if (Config::sync)
    mVioCore->processSync();
}

void SLAM::setAcc(const uint64_t& ns, float* acc) {}

void SLAM::setGyr(const uint64_t& ns, float* gyr) {}

}  //namespace toy
