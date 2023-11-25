#pragma once
#include "Camera.h"
#include <opencv2/opencv.hpp>

namespace toy {
class PinholeRadialTangential : public Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeRadialTangential(CameraInfo* cameraInfo)
    : Camera(cameraInfo) {
    mK = (cv::Mat_<double>(3, 3) << mFx, 0.0, mCx, 0.0, mFy, mCy, 0.0, 0.0, 1.0);
    mD = (cv::Mat_<double>(1, 5) << mD0, mD1, mD2, mD3, mD4);
  }
  PinholeRadialTangential(PinholeRadialTangential* src)
    : Camera(src) {}

  ~PinholeRadialTangential() = default;

  Camera* clone() override {
    PinholeRadialTangential* out = new PinholeRadialTangential(this);
    return out;
  };

  void project(Eigen::Vector3d& _xyz, Eigen::Vector2d& uv_) override {}

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& undists) override {
    cv::undistortPoints(pts, undists, mK, mD);
  }
};
}  //namespace toy