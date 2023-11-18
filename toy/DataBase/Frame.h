#pragma once
#include <array>
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

  std::array<ImagePyramid*, 2> mImagePyramids;
  std::array<Camera*, 2>       mCameras;
  std::array<Feature*, 2>      mFeatures;

  //P is SE3
  Sophus::SE3d Pwb;
  Sophus::SE3d Pbc0;
  Sophus::SE3d Pbc1;

public:
  ImagePyramid* getImagePyramid(int i) { return mImagePyramids[i]; }

  Camera* getCamera(int i) { return mCameras[i]; }
  void    setCameras(Camera* cam0, Camera* cam1) {
    mCameras[0] = cam0;
    mCameras[1] = cam1;
  }

  Feature* getFeature(int i) { return mFeatures[i]; }
};
}  //namespace db
}  //namespace toy