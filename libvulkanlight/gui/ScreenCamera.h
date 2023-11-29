#pragma once

#include "GraphicsCamera.h"

namespace vkl {

class ScreenCamera : public GraphicsCamera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScreenCamera() = delete;

  /**
   * @brief
   * @param intrinsics calibrated camera's  width, height, fx, fy, cx, cy
   * @param distortion nullptr
   * @param orientation device orientation
   * @param model
   */
  ScreenCamera(GraphcisAPI api,
               int         screenW,
               int         screenH,
               float*      intrinsics,
               float*      distortion,
               int         orientation);

  virtual ~ScreenCamera();

  void fitDeviceFromCamera();
  void onWindowResized(int w, int h, int newOrientation = 0) override;

  Eigen::Matrix4f getCamUnprojection() {
    float cw = camParameters[0];
    float ch = camParameters[1];

    float cfx = camParameters[2];
    float cfy = camParameters[3];
    float ccx = camParameters[4];
    float ccy = camParameters[5];

    float xRatio = n / cfx;
    float yRatio = n / cfy;

    float cr = (cw - ccx) * xRatio;
    float cl = -ccx * xRatio;

    float ct = (ccy)*yRatio;
    float cb = -(ch - ccy) * yRatio;

    //width, height, fx, fy, cx, cy

    float rml = cr - cl;
    float rpl = cr + cl;

    float tmb = ct - cb;
    float tpb = ct + cb;

    float n2  = 2.0f * n;
    float fpn = f + n;
    float fmn = f - n;

    Eigen::Matrix4f P;
    // clang-format off
  switch (api) {
  case vkl::GraphicsCamera::OPENGL:
    P
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , -n2 / tmb ,   tpb / tmb, 0
      ,  0       , 0         ,   fpn/fmn  , -n2 * f / fmn
      ,  0       , 0         ,   1        , 0;        
    break;
  case vkl::GraphicsCamera::VULKAN:
    P
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , n2 / tmb  ,  -tpb / tmb, 0
      ,  0       , 0         ,   f/fmn    , -n * f / fmn
      ,  0       , 0         ,   1        , 0;        
    break;
  case vkl::GraphicsCamera::METAL:
     P
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , -n2 / tmb ,   tpb / tmb, 0
      ,  0       , 0         ,   f/fmn    , -n * f / fmn
      ,  0       , 0         ,   1        , 0;      
    break;
  default:
    break;
  }
  return P.inverse();
  }


protected:

  void setInputCallback() override;

public:

protected:

  float camParameters[6];
};

}  //namespace vkl