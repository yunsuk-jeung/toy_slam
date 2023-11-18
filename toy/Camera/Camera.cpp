#include "Camera.h"
#include "PinholeRadTan.h"

namespace toy {
Camera::Camera(CameraInfo* cameraInfo)
    : w{(float)cameraInfo->w}
    , h{(float)cameraInfo->w}
    , fx{cameraInfo->intrinsics[0]}
    , fy{cameraInfo->intrinsics[1]}
    , cx{cameraInfo->intrinsics[2]}
    , cy{cameraInfo->intrinsics[3]}
    , k0{cameraInfo->distortions[0]}
    , k1{cameraInfo->distortions[1]}
    , k2{cameraInfo->distortions[2]}
    , k3{cameraInfo->distortions[3]}
    , k4{cameraInfo->distortions[4]} {}

Camera* CameraFactory::createCamera(CameraInfo* camInfo) {
  switch (camInfo->cameraModel) {
  case 0 /*pinhole*/:
    if (camInfo->distortionModel == 0) {
      return new PinholeRadTan(camInfo);
    }

    break;
  default:
    break;
  }

  return nullptr;
}

Camera::~Camera() {}

}  //namespace toy