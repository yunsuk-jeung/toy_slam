#pragma once
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "macros.h"
#include "types.h"
namespace toy {
class Camera {
public:
  USING_SMART_PTR(Camera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Camera(CameraInfo* camInfo);
  Camera(Camera* src);

  virtual ~Camera();
  virtual Camera* clone() = 0;

  virtual void        project(Eigen::Vector3d& xyz, Eigen::Vector2d& uv);
  virtual cv::Point2d project(Eigen::Vector3d& xyz);
  virtual void        distort(const Eigen::Vector2d& input, Eigen::Vector2d& output) = 0;

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& upts) = 0;

protected:
  int    cameraModel;
  double mW, mH;
  double mFx, mFy, mCx, mCy;
  double mD0, mD1, mD2, mD3, mD4;

  double mInvFx, mInvFy, mInvCx, mInvCy;
  bool   mIsDistortion;

  cv::Mat                     mK, mD;
  Eigen::Matrix3d             mEK;
  Eigen::Matrix<double, 5, 1> mED;
};

class CameraFactory {
public:
  static Camera* createCamera(CameraInfo* camInfo);
};
}  //namespace toy