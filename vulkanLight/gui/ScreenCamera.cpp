#include <cmath>  // tan
#include <iostream>

#include "ScreenCamera.h"

namespace vkl {
namespace {
static const float PI = std::acos(-1);
inline float       to_rad(float degree) {
  return degree * PI / 180.0f;
}
}  //namespace

ScreenCamera::ScreenCamera(GraphcisAPI api,
                           int         screenW,
                           int         screenH,
                           float*      intrinsics,  //w, h, fx fy, cx, cy
                           float*      distortion,
                           int         orientation)
  : GraphicsCamera(api,
                   screenW,
                   screenH,
                   ProjectionMode::PERSPECTIVE,
                   &intrinsics[2],
                   distortion,
                   orientation,
                   DistortionModel::RADTAN) {
  memcpy(camParameters, intrinsics, sizeof(float) * 6);

  fitDeviceFromCamera();
}

ScreenCamera::~ScreenCamera() {}

void ScreenCamera::fitDeviceFromCamera() {
  //return;
  const float& tw  = camParameters[0];
  const float& th  = camParameters[1];
  const float& fx_ = camParameters[2];
  const float& fy_ = camParameters[3];
  const float& tcx = camParameters[4];
  const float& tcy = camParameters[5];

  const float texAspect = tw / th;

  const float ww = windowSize.x();
  const float wh = windowSize.y();

  const float winAspect = ww / wh;

  switch (orientation) {
  case 0:  //landcape
  {
    float rw = tcx > (tw * 0.5f) ? tw - tcx : tcx;
    float rh = tcy > (th * 0.5f) ? th - tcy : tcy;

    float rw_ww = rw / ww;
    float rh_wh = rh / wh;
    float cx_, cy_;

    if (rw_ww < rh_wh) {
      cx_ = rw;
      cy_ = cx_ / winAspect;
    }
    else {
      cy_ = rh;
      cx_ = cy_ * winAspect;
    }

    windowSize << cx_ * 2, cy_ * 2;
    setIntrinsic(fx_, fy_, cx_, cy_);
    break;
  }
  case 1:  //portrait
    break;
  default:
    break;
  }
}

void ScreenCamera::onWindowResized(int w, int h, int newOrientation) {
  windowSize << w, h;
  orientation = newOrientation;

  fitDeviceFromCamera();
}

void ScreenCamera::setInputCallback() {}

}  //namespace vkl
