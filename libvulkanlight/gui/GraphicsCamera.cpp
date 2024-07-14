#include <cmath>  // tan
#include <iostream>

#include "GraphicsCamera.h"
namespace vkl {
namespace {
static const float PI = std::acos(-1);
inline float       to_rad(float degree) {
  return degree * PI / 180.0f;
}
}  //namespace

GraphicsCamera::GraphicsCamera(GraphcisAPI     _api,
                               int             w,
                               int             h,
                               ProjectionMode  proj,
                               float*          intrinsics,
                               float*          distortion,
                               int             orientation_,
                               DistortionModel model)
  : center{Eigen::Vector3f(0, 0, 0)}
  , distance{10.0f}
  , longitude{45.0f}
  , latitude{45.0f}
  , api{_api}
  , projMode{proj}
  , projMatMethod{FOV}
  , hfov{60.0f}
  , inverseAspect{0.0f}
  , n{0.1f}
  , f{5000.0f}
  , orientation{orientation_} {
  camFront << 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f;
  cam.M = Eigen::Matrix4f::Identity();
  cam.V = Eigen::Matrix4f::Identity();

  updateV();

  onWindowResized(w, h, orientation);

  if (intrinsics)
    setIntrinsic(intrinsics);
  else
    setFov(hfov);

  setInputCallback();
}

GraphicsCamera::~GraphicsCamera() {}

void GraphicsCamera::onWindowResized(int w, int h, int newOrientation) {
  windowSize << w, h;
  inverseAspect = (float)h / (float)w;
  orientation   = newOrientation;

  if (projMatMethod == FOV)
    setFov(hfov);
}

void GraphicsCamera::setFov(float _hfov) {
  projMatMethod = FOV;
  hfov          = _hfov;

  float tmp = std::tan(hfov / 360.0f * PI);

  l = -tmp * n;
  r = tmp * n;

  t = inverseAspect * r;
  b = -inverseAspect * r;

  updateP();
}

void GraphicsCamera::setIntrinsic(float* intrinsics) {
  setIntrinsic(intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
}

void GraphicsCamera::setIntrinsic(float _fx, float _fy, float _cx, float _cy) {
  projMatMethod = INTRINSIC;

  fx = _fx;
  fy = _fy;
  cx = _cx;
  cy = _cy;

  float w = windowSize.x();
  float h = windowSize.y();

  float xRatio = n / fx;
  float yRatio = n / fy;

  r = (w - cx) * xRatio;
  l = -cx * xRatio;

  t = (cy)*yRatio;
  b = -(h - cy) * yRatio;

  updateP();
}

void GraphicsCamera::setM(Eigen::Matrix4f& M) {
  cam.M = M;
}

void GraphicsCamera::setV(Eigen::Matrix4f& V) {
  cam.V = V;
}

void GraphicsCamera::setP(Eigen::Matrix4f& P) {
  cam.P = P;
}

Eigen::Matrix4f GraphicsCamera::getCamUnprojection() {
  return cam.P.inverse();
}

//void GraphicsCamera::setDistortion(float* distortion) {}

//void GraphicsCamera::setDistortion(float k1, float k2, float k3, float k4,
//float k5) {}

/**
 * @brief  projection matrix is derived when camera is looking at Z-direction
 * O -- X
 * |
 * Y
 */
void GraphicsCamera::updateP() {
  float rml = r - l;
  float rpl = r + l;

  float tmb = t - b;
  float tpb = t + b;

  float n2  = 2.0f * n;
  float fpn = f + n;
  float fmn = f - n;
  // clang-format off
  switch (api) {
  case vkl::GraphicsCamera::OPENGL:
    cam.P 
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , -n2 / tmb ,   tpb / tmb, 0
      ,  0       , 0         ,   fpn/fmn  , -n2 * f / fmn
      ,  0       , 0         ,   1        , 0;        
    break;
  case vkl::GraphicsCamera::VULKAN:
    cam.P 
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , n2 / tmb  ,  -tpb / tmb, 0
      ,  0       , 0         ,   f/fmn    , -n * f / fmn
      ,  0       , 0         ,   1        , 0;        
    break;
  case vkl::GraphicsCamera::METAL:
     cam.P 
      << n2 / rml, 0         ,  -rpl / rml, 0
      ,  0       , -n2 / tmb ,   tpb / tmb, 0
      ,  0       , 0         ,   f/fmn    , -n * f / fmn
      ,  0       , 0         ,   1        , 0;      
    break;
  default:
    break;
  }
  // clang-format on
}

void GraphicsCamera::updateV() {
  //orientation -> pose of current camera with lat/long rotation only
  Eigen::Matrix3f orientation = (Eigen::AngleAxisf(to_rad(longitude),
                                                   Eigen::Vector3f::UnitZ())
                                 * Eigen::AngleAxisf(-to_rad(latitude),
                                                     Eigen::Vector3f::UnitY()))
                                  .toRotationMatrix();
  cam.V.topLeftCorner<3, 3>()  = (orientation * camFront).transpose();
  cam.V.topRightCorner<3, 1>() = -cam.V.topLeftCorner<3, 3>()
                                 * (orientation * Eigen::Vector3f(distance, 0, 0)
                                    + center);
}

void GraphicsCamera::onMouseClick(int btn, int px, int py) {
  mouseX = px;
  mouseY = py;
}

void GraphicsCamera::onMouseDrag(int btn, int px, int py) {
  float delx = px - mouseX;
  float dely = py - mouseY;
  float relx = delx / static_cast<float>(windowSize.x());
  float rely = dely / static_cast<float>(windowSize.x());
  mouseX     = px;
  mouseY     = py;

  constexpr float scaleR = 1.5f;
  constexpr float scaleT = 1.0f;

  Eigen::Vector2f reldD(relx, rely);

  switch (btn) {
  case 1:  //rotation

    reldD *= 180.0f * scaleR;  //to degree
    latitude += reldD.y();

    //clamp
    //latitude = (180.0f < latitude) ? latitude - 360.0f : latitude;
    //latitude = (latitude < -180.0f) ? latitude + 360.0f : latitude;

    latitude = (90.0f < latitude) ? 90.0f : latitude;
    latitude = (latitude < -90.0f) ? -90.0f : latitude;

    //if (latitude > 0) {
    //	longitude -= relative_delta.x();
    //}
    //else {
    //	longitude += relative_delta.x();
    //}

    longitude -= reldD.x();
    longitude = (360.0f < longitude) ? (longitude - 360.0f) : longitude;
    longitude = (longitude < 0.0f) ? (longitude + 360.0f) : longitude;
    break;
  case 0:  //move
  {
    reldD *= distance * scaleT;
    Eigen::Matrix3f orientation = (Eigen::AngleAxisf(to_rad(longitude),
                                                     Eigen::Vector3f::UnitZ())
                                   * Eigen::AngleAxisf(-to_rad(latitude),
                                                       Eigen::Vector3f::UnitY()))
                                    .toRotationMatrix();
    Eigen::Vector3f translation = orientation
                                  * (Eigen::Vector3f(0.0f, -reldD.x(), reldD.y()));
    center += translation;
    break;
  }
  case 2:  //nothing
  {
    break;
  }
  default:
    break;
  }
  updateV();
}

void GraphicsCamera::onMouseWheel(int scroll) {
  if (scroll > 0)  //zoom in
  {
    if (distance < 0.05f)
      return;

    distance *= 0.9f;
  }
  else  //zoom out.
  {
    if ((f * 0.4) < distance)
      return;
    distance *= 1.1f;
  }

  updateV();
}

void GraphicsCamera::setInputCallback() {
  auto clickFunc = std::bind(&GraphicsCamera::onMouseClick,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3);

  this->registerMouseClick(clickFunc);

  auto dragFunc = std::bind(&GraphicsCamera::onMouseDrag,
                   this,
                   std::placeholders::_1,
                   std::placeholders::_2,
                   std::placeholders::_3);

  this->registerMouseDrag(dragFunc);

  auto func2 = std::bind(&GraphicsCamera::onMouseWheel, this, std::placeholders::_1);
  this->registerMouseWheel(func2);
}

}  //namespace vkl
