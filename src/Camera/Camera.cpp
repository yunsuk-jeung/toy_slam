#include "Camera.h"
#include "PinholeRadTan.h"

namespace toy {
Camera::Camera(CameraInfo* cameraInfo)
  : mW{(float)cameraInfo->w}
  , mH{(float)cameraInfo->w}
  , mFx{cameraInfo->intrinsics[0]}
  , mFy{cameraInfo->intrinsics[1]}
  , mCx{cameraInfo->intrinsics[2]}
  , mCy{cameraInfo->intrinsics[3]}
  , mD0{cameraInfo->distortions[0]}
  , mD1{cameraInfo->distortions[1]}
  , mD2{cameraInfo->distortions[2]}
  , mD3{cameraInfo->distortions[3]}
  , mD4{cameraInfo->distortions[4]}
  , mIsDistortion{false} {
  mInvFx = 1.0 / mFx;
  mInvFy = 1.0 / mFy;
  mInvCx = -mCx / mFx;
  mInvCy = -mCy / mFy;

  if (mD0 != 0.0 || mD1 != 0.0 || mD2 != 0.0 || mD3 != 0.0) {
    mIsDistortion = true;
  }
}

Camera::Camera(Camera* src) {
  this->mW            = src->mW;
  this->mH            = src->mH;
  this->mFx           = src->mFx;
  this->mFy           = src->mFy;
  this->mCx           = src->mCx;
  this->mCy           = src->mCy;
  this->mD0           = src->mD0;
  this->mD1           = src->mD1;
  this->mD2           = src->mD2;
  this->mD3           = src->mD3;
  this->mD4           = src->mD4;
  this->mInvFx        = src->mInvFx;
  this->mInvFy        = src->mInvFy;
  this->mInvCx        = src->mInvCx;
  this->mInvCy        = src->mInvCy;
  this->mIsDistortion = src->mIsDistortion;
  this->mK            = src->mK.clone();
  this->mD            = src->mD.clone();
  this->mEK           = src->mEK;
  this->mED           = src->mED;
}

Camera* CameraFactory::createCamera(CameraInfo* camInfo) {
  switch (camInfo->cameraModel) {
  case 0 /*pinhole*/:
    if (camInfo->distortionModel == 0) {
      return new PinholeRadialTangential(camInfo);
    }

    break;
  default:
    break;
  }

  return nullptr;
}

Camera::~Camera() {}

void Camera::project(Eigen::Vector3d& xyz, Eigen::Vector2d& uv) {
  Eigen::Vector2d nuv(xyz.x() / xyz.z(), xyz.y() / xyz.z());

  if (mIsDistortion) {
    Eigen::Vector2d dnuv;
    distort(nuv, dnuv);
    nuv += dnuv;
  }
  uv << mFx * nuv.x() + mCx, mFy * nuv.y() + mCy;
}

cv::Point2d Camera::project(Eigen::Vector3d& xyz) {
  Eigen::Vector2d nuv(xyz.x() / xyz.z(), xyz.y() / xyz.z());

  if (mIsDistortion) {
    Eigen::Vector2d dnuv;
    distort(nuv, dnuv);
    nuv += dnuv;
  }
  return cv::Point2d(mFx * nuv.x() + mCx, mFy * nuv.y() + mCy);
}

}  //namespace toy