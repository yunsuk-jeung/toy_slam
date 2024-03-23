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

void SLAM::setNewImages(std::vector<ImageData>& images) {
  const auto imageCount = images.size();

  std::vector<db::ImagePyramid::Ptr> imagePyramids;
  imagePyramids.reserve(imageCount);

  for (auto& image : images) {
    imagePyramids.emplace_back(std::make_shared<db::ImagePyramid>(image));
  }

  db::ImagePyramidSet::Ptr imagePyramidSet = std::make_shared<db::ImagePyramidSet>(
    imagePyramids);

  mVioCore->insert(imagePyramidSet);

  if (Config::sync)
    mVioCore->processSync();
}

void SLAM::setAcc(const uint64_t& ns, float* acc) {}

void SLAM::setGyr(const uint64_t& ns, float* gyr) {}

}  //namespace toy
