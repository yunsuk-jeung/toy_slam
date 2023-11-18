#pragma once
#include <memory>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
namespace toy {
class Camera;
namespace db {
class ImagePyramid;
class LocalMap;
class Feature;
class Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class MemoryPointerPool;

  void setPbc(float*, float*);

protected:
  Frame() = delete;
  Frame(ImagePyramid*);
  ~Frame();

protected:
  int mId;

  ImagePyramid* mImagePyramid0;
  ImagePyramid* mImagePyramid1;

  Camera* mCam0;
  Camera* mCam1;

  Feature* mFeature0;
  Feature* mFeature1;

  //P is SE3
  Sophus::SE3d Pwb;
  Sophus::SE3d Pbc0;
  Sophus::SE3d Pbc1;

public:
  ImagePyramid* getImagePyramid0() { return mImagePyramid0; }
  ImagePyramid* getImagePyramid1() { return mImagePyramid1; }

  void setCameras(Camera* cam0, Camera* cam1) {
    mCam0 = cam0;
    mCam1 = cam1;
  }

  Feature* getFeature0() { return mFeature0; }
  Feature* getFeature1() { return mFeature1; }
};
}  //namespace db
}  //namespace toy