#include "Slam.h"

namespace toy {

SLAM::SLAM(){};

SLAM::~SLAM(){};

void SLAM::init(const std::string& configFile) {}

void SLAM::setNewImage(ImageType type,
                       uint64_t& ns,
                       uint8_t*  buffer,
                       int       lenght,
                       int       width,
                       int       height) {}

void SLAM::setAcc(uint64_t& ns, float* acc) {}

void SLAM::setGyr(uint64_t& ns, float* gyr) {}

}  //namespace toy
