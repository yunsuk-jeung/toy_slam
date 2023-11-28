#pragma once
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "macros.h"
#include "types.h"
namespace toy {
class Camera {
public:
  TOY_SMART_PTR(Camera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Camera(CameraInfo* camInfo);
  Camera(Camera* src);

  virtual ~Camera();
  virtual Camera* clone() = 0;

  virtual void project(Eigen::Vector3d& _xyz, Eigen::Vector2d& uv_) = 0;
  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& upts)      = 0;

protected:
  int    cameraModel;
  double mW, mH;
  double mFx, mFy, mCx, mCy;
  double mD0, mD1, mD2, mD3, mD4;

  double mInvFx, mInvFy, mInvCx, mInvCy;
  bool   mIsDistortion;

  cv::Mat mK, mD;
};

class CameraFactory {
public:
  static Camera* createCamera(CameraInfo* camInfo);
};
}  //namespace toy