#pragma once
#include "Camera.h"
#include <opencv2/opencv.hpp>

namespace toy {
class PinholeRadialTangential : public Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeRadialTangential() = default;
  PinholeRadialTangential(CameraInfo* cameraInfo)
    : Camera(cameraInfo) {
    mK = (cv::Mat_<double>(3, 3) << mFx, 0.0, mCx, 0.0, mFy, mCy, 0.0, 0.0, 1.0);
    mD = (cv::Mat_<double>(1, 5) << mD0, mD1, mD2, mD3, mD4);

    mEK << mFx, 0.0, mCx, 0.0, mFy, mCy, 0.0, 0.0, 1.0;
    mED << mD0, mD1, mD2, mD3, mD4;
  }
  PinholeRadialTangential(PinholeRadialTangential* src)
    : Camera(src) {}

  ~PinholeRadialTangential() = default;

  Camera* clone() override {
    PinholeRadialTangential* out = new PinholeRadialTangential(this);
    return out;
  };

  void distort(const Eigen::Vector2d& nuv, Eigen::Vector2d& dnuv) override {
    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = nuv.x() * nuv.x();
    my2_u = nuv.y() * nuv.y();
    mxy_u = nuv.x() * nuv.y();

    rho2_u = mx2_u + my2_u;

    rad_dist_u = mD0 * rho2_u + mD1 * rho2_u * rho2_u;

    dnuv << nuv.x() * rad_dist_u + 2.0 * mD2 * mxy_u + mD3 * (rho2_u + 2.0 * mx2_u),
      nuv.y() * rad_dist_u + 2.0 * mD3 * mxy_u + mD2 * (rho2_u + 2.0 * my2_u);
  }

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& undists) override {
    cv::undistortPoints(pts, undists, mK, mD);
  }
};
}  //namespace toy