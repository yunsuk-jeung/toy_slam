#pragma once
namespace io {
struct CamInfo {
  int   type{0};
  int   w{0};
  int   h{0};
  float intrinsic[4]  = {0};
  float distortion[5] = {0};
  float bMc[16]       = {0};
};

struct ImuInfo {
  float gyrNoiseDensity{0};
  float gyrRandomWalk{0};
  float accNoiseDensity{0};
  float accRandomWalk{0};
};
}  //namespace io