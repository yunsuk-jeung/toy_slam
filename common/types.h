#pragma once
#include <cstdint>
#include <vector>
enum ImageType { NONE = -1, CAM0 = 0, CAM1 = 1, DEPTH = 2 };
struct ImageData {
  int      type   = -1;
  int      format = 0;
  uint64_t ns     = 0;
  uint8_t* buffer = nullptr;
  int      w      = 0;
  int      h      = 0;
};

struct CameraInfo {
  int                id;
  int                w;                //2
  int                h;                //3
  int                cameraModel;      //4
  std::vector<float> intrinsics;       //8
  int                distortionModel;  //9
  std::vector<float> distortions;      //14
  std::vector<float> Mbc;              //30

  CameraInfo()
    : id{0}
    , w{0}
    , h{0}
    , cameraModel{0}
    , intrinsics{}
    , distortionModel{0}
    , distortions(5, 0)
    , Mbc(16, 0) {}

  ~CameraInfo() {}
};

struct ImuInfo {
  float gyrNoiseDensity{0};
  float gyrRandomWalk{0};
  float accNoiseDensity{0};
  float accRandomWalk{0};
};
