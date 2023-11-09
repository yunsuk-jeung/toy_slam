#include "config.h"
#include "Frame.h"
#include "Slam.h"
#include "Vio.h"

namespace toy {

SLAM::SLAM() : vioUPtr{nullptr} {};

SLAM::~SLAM(){};

void SLAM::init(const std::string& configFile) {
  Config::parseConfig(configFile);
  vioUPtr = std::make_unique<Vio>();
}

void SLAM::setNewImage(ImageType type,
                       uint64_t& ns,
                       uint8_t*  buffer,
                       int       lenght,
                       int       width,
                       int       height) {}

void SLAM::setAcc(uint64_t& ns, float* acc) {}

void SLAM::setGyr(uint64_t& ns, float* gyr) {}

}  //namespace toy
