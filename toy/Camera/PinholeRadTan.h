#pragma once
#include "Camera.h"
namespace toy {
class PinholeRadTan : public Camera {
public:
  PinholeRadTan(CameraInfo* cameraInfo) : Camera(cameraInfo) {}
  ~PinholeRadTan() = default;

  void         project(Eigen::Vector3d& _xyz, Eigen::Vector2d& uv_) override {}
  virtual void undistortPoints(std::vector<cv::Point2f>&     pts,
                               std::vector<Eigen::Vector2d>& upts) override {}
};
}  //namespace toy