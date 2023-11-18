#pragma once
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "types.h"
namespace toy {
class Camera {
public:
  Camera(CameraInfo* camInfo);
  virtual ~Camera();

  virtual void project(Eigen::Vector3d& _xyz, Eigen::Vector2d& uv_) = 0;
  virtual void undistortPoints(std::vector<cv::Point2f>&     pts,
                               std::vector<Eigen::Vector2d>& upts)  = 0;

protected:
  double w, h;
  double fx, fy, cx, cy;
  double k0, k1, k2, k3, k4;
};

class CameraFactory {
public:
  static Camera* createCamera(CameraInfo* camInfo);
};
}  //namespace toy