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

  if (mD0 != 0.0 || mD1 != 0.0 || mD2 != 0.0 || mD3 != 0.0) { mIsDistortion = true; }
}

Camera* CameraFactory::createCamera(CameraInfo* camInfo) {
  switch (camInfo->cameraModel) {
  case 0 /*pinhole*/:
    if (camInfo->distortionModel == 0) { return new PinholeRadialTangential(camInfo); }

    break;
  default:
    break;
  }

  return nullptr;
}

Camera::~Camera() {}

}  //namespace toy