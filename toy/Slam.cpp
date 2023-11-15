#include "config.h"
#include "ToyLogger.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Map.h"
#include "Vio.h"
#include "Slam.h"

namespace toy {

SLAM::SLAM() : vio{nullptr} {};

SLAM::~SLAM() {
  delete vio;
  vio = nullptr;
};

void SLAM::setSensorInfo(float* cam0, float* cam1, float* imu) {
  memcpy(Config::Vio::camInfo0, cam0, sizeof(float) * 12);
  memcpy(Config::Vio::camInfo1, cam1, sizeof(float) * 12);
  if (imu) {
    memcpy(Config::Vio::imuInfo, imu, sizeof(float) * 4);
  }
}

void SLAM::prepare(const std::string& configFile) {
  Config::parseConfig(configFile);
  vio = new Vio();
  vio->prepare();
}

void SLAM::setNewImage(const int       type_,
                       const int       cvFormat,
                       const uint64_t& ns,
                       uint8_t*        buffer,
                       const int       w,
                       const int       h) {

  auto type = static_cast<ImageType>(type_);

  auto* imagePyramid = new db::ImagePyramid(type, cvFormat, buffer, w, h);
  vio->insert(imagePyramid);

  ToyLogD("curr type : {}, format : {}",type_, cvFormat);
  //std::cout << type << std::endl;
  if (Config::sync) vio->process();
}

void SLAM::setAcc(const uint64_t& ns, float* acc) {}

void SLAM::setGyr(const uint64_t& ns, float* gyr) {}

}  //namespace toy
