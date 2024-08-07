#pragma once

#include <string>

#include "types.h"
#include "Singleton.h"

class CameraInfo;
class ImuInfo;
namespace toy {
class VioCore;
class SLAM : public Singleton<SLAM> {
public:
  friend class Singleton<SLAM>;
  void setSensorInfo(CameraInfo* cam0, CameraInfo* cam1, ImuInfo* imu = nullptr);
  void prepare(const std::string& configFile);

  void setNewImages(std::vector<ImageData>& images);

  void setAcc(const uint64_t& ns, float* acc);
  void setGyr(const uint64_t& ns, float* gyr);

private:
  SLAM();
  ~SLAM() override;

  VioCore* mVioCore;
};
}  //namespace toy