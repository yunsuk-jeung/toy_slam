#pragma once
#include <array>
#include <memory>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include "usings.h"
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

  void setLbc(float*, float*);

protected:
  Frame() = delete;
  Frame(ImagePyramid*);
  ~Frame();

protected:
  int mId;

  std::array<ImagePyramid*, 2> mImagePyramids;
  std::array<Camera*, 2>       mCameras;
  std::array<Feature*, 2>      mFeatures;

  //L : lie
  Eigen::Vector6d                mLwb;
  std::array<Eigen::Vector6d, 2> mLbcs;

public:
  ImagePyramid* getImagePyramid(int i) { return mImagePyramids[i]; }

  Camera* getCamera(int i) { return mCameras[i]; }
  void    setCameras(Camera* cam0, Camera* cam1) {
    mCameras[0] = cam0;
    mCameras[1] = cam1;
  }

  Feature* getFeature(int i) { return mFeatures[i]; }

  Eigen::Vector6d& getLbc(int i) { return mLbcs[i]; }
};
}  //namespace db
}  //namespace toy