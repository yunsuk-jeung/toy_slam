#pragma once
#include <functional>
#include <iostream>

#include "types.h"
#include "SensorInfo.h"

namespace io {

class Sensor {
public:
  using ImageCallback = std::function<void(ImageData&, ImageData&)>;

  using ImuCallback = std::function<void(const uint64_t&, float*)>;

  Sensor()          = default;
  virtual ~Sensor() = default;

  virtual void prepare() = 0;
  virtual void start()   = 0;
  virtual void stop()    = 0;

  virtual void getInfo(CameraInfo* info0, CameraInfo* info1) = 0;

  void registerImageCallback(ImageCallback cb) { mImageCallBack = cb; }
  void registerAccCallback(ImuCallback cb) { mAccCallback = cb; }
  void registerGyrCallback(ImuCallback cb) { mGyrCallback = cb; }

  bool isSimulator() const { return mIsSimulator; };

protected:
  ImageCallback mImageCallBack = [](ImageData& imageData0, ImageData& ImageData1) {
    std::cout << "You forgot to register image Callback\n";
  };

  ImuCallback mAccCallback = [](const uint64_t& ns, float* imu) {
    std::cout << "u forgot to register acc Callback" << std::endl;
  };

  ImuCallback mGyrCallback = [](const uint64_t& ns, float* imu) {
    std::cout << "u forgot to register gyr Callback" << std::endl;
  };

  CameraInfo mCamInfo0;
  CameraInfo mCamInfo1;
  ImuInfo    mImuInfo;

  int  mImageFormat = 0;
  bool mIsSimulator = false;
};
}  //namespace io