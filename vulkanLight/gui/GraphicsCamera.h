#pragma once

#include <Eigen/Dense>
#include <array>
#include "InputCallback.h"
#include "shaders/ShaderTypes.h"
#include "core/types.h"
namespace vkl {

class GraphicsCamera : public InputCallback {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum GraphcisAPI { OPENGL, VULKAN, METAL };

  enum ProjectionMode {
    PERSPECTIVE,
    ORTHOGRAPHIC,
  };

  GraphicsCamera() = delete;

  GraphicsCamera(GraphcisAPI     api,
                 int             w,
                 int             h,
                 ProjectionMode  proj,
                 float*          intrinsics,
                 float*          distortion,
                 int             orientation,
                 DistortionModel model = DistortionModel::RADTAN);

  virtual ~GraphicsCamera();

  /**
   * @brief when window size and camera's image size is different.
   * @param externalCamInfo : external camera's w, h, fx, fy ,cx ,cy
   */
  virtual void onWindowResized(int w, int h, int newOrientation = 0);

  void setFov(float hfov);

  void setIntrinsic(float* intrinsics);
  void setIntrinsic(float fx, float fy, float cx, float cy);

  void setM(Eigen::Matrix4f& M);
  void setV(Eigen::Matrix4f& V);
  void setP(Eigen::Matrix4f& P);

  //void setDistortion(float* distortion);
  //void setDistortion(float k1, float k2, float k3, float k4, float k5);
  virtual Eigen::Matrix4f getCamUnprojection();

protected:
  virtual void updateP();
  void         updateV();

  virtual void onMouseClick(int btn, int px, int py);
  virtual void onMouseDrag(int btn, int px, int py);
  virtual void onMouseWheel(int scroll);
  //void         onKeyPressed(int key);
  //void         onKeyDown(int key);
  //void         onKeyRelease(int key);

  virtual void setInputCallback();

public:
  CameraUniform cam;

protected:
  Eigen::Matrix3f camFront;

  //MouseEvents
  Eigen::Vector2i windowSize;
  int             mouseX;
  int             mouseY;

  //View
  Eigen::Vector3f center;
  float           distance;
  float           longitude;
  float           latitude;

  //api
  GraphcisAPI api = VULKAN;

  //projection
  ProjectionMode projMode;

  enum ProjectionMatrixMethod {
    FOV,
    INTRINSIC,
  } projMatMethod;

  //use intrinsics to create P
  float fx, fy, cx, cy;

  //use fov to create P
  float hfov;
  float inverseAspect;

  //distortion
  float k1, k2, k3, k4, k5;

  //create P with below params
  float n, f, l, r, b, t;

  int orientation;
};

}  //namespace vkl