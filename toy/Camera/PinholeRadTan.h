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

  ~PinholeRadialTangential() = default;

  void project(Eigen::Vector3d& _xyz, Eigen::Vector2d& uv_) override {}

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& undists) override {
    cv::undistortPoints(pts, undists, mK, mD);
  }

  //virtual void undistortPoint(const cv::Point2f& pt, Eigen::Vector2d& undist)
  //{
  //  Eigen::Vector3d n_xyz = inverseProjection(pt);  //normalized xyz
  //}

  //Eigen::Vector3d inverseProjection(const cv::Point2f& pt) {
  //  Eigen::Vector2d np;
  //  np << mInvFx * (double)pt.x + mInvCx, mInvFy * (double)pt.y + mInvCy;

  //if (mIsDistortion) {
  //}

  //return Eigen::Vector3d(np.x(), np.y(), 1.0);
  //}
};
}  //namespace toy