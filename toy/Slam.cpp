#include "Slam.h"
#include "config.h"
#include "Vio.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Map.h"
namespace toy {

SLAM::SLAM() : vioUPtr{nullptr} {};

SLAM::~SLAM(){};

void SLAM::prepare(const std::string& configFile) {
  Config::parseConfig(configFile);
  vioUPtr = std::make_unique<Vio>();
  vioUPtr->prepare();
}

void SLAM::setNewImage(ImageType   type,
                       ImageFormat format,
                       uint64_t&   ns,
                       uint8_t*    buffer,
                       int         l,
                       int         w,
                       int         h) {
  auto* imagePyramid = new db::ImagePyramid(type, format, buffer, l, w, h);
}

void SLAM::setAcc(uint64_t& ns, float* acc) {}

void SLAM::setGyr(uint64_t& ns, float* gyr) {}

}  //namespace toy
