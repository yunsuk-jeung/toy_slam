#pragma once
#include <array>
#include <memory>
#include <map>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "macros.h"
#include "Factor.h"

namespace toy {
class Camera;
namespace db {
class ImagePyramid;
class ImagePyramidSet;
class LocalMap;
class Feature;
class MapPoint;
class Frame {
public:
  USING_SMART_PTR(Frame);
  DELETE_COPY_CONSTRUCTORS(Frame);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame(std::shared_ptr<ImagePyramidSet> set);
  ~Frame();
  Frame(Frame* src);

  Frame::Ptr clonePtr();

  void setCameras(Camera* cam0, Camera* cam1);
  //void setLbc(float*, float*);
  void setSbc(float*, float*);

  void addMapPointFactor(std::shared_ptr<db::MapPoint> mp, ReprojectionFactor factor);

protected:

protected:
  using MapPointFactorMap = std::map<std::weak_ptr<db::MapPoint>,
                                     ReprojectionFactor,
                                     std::owner_less<std::weak_ptr<db::MapPoint>>>;
  static int globalId;
  int        mId;

  std::array<std::unique_ptr<ImagePyramid>, 2> mImagePyramids;
  std::array<std::unique_ptr<Camera>, 2>       mCameras;
  std::array<std::unique_ptr<Feature>, 2>      mFeatures;

  MapPointFactorMap mMapPointFactorMap;

  ////L : lie
  //Eigen::Vector6d                mLwb;
  //std::array<Eigen::Vector6d, 2> mLbcs;
  //Eigen::Vector6d&   getLbc(int i) { return mLbcs[i]; }

  //S : se3
  Sophus::SE3d                mSwb;
  std::array<Sophus::SE3d, 2> mSbcs;

public:
  const int          id() const { return mId; }
  ImagePyramid*      getImagePyramid(int i) { return mImagePyramids[i].get(); }
  Camera*            getCamera(int i) { return mCameras[i].get(); }
  Feature*           getFeature(int i) { return mFeatures[i].get(); }
  void               setSwb(const Sophus::SE3d& Swb) { mSwb = Swb; }
  Sophus::SE3d&      getSwb() { return mSwb; }
  Sophus::SE3d       getSwc(int i) { return mSwb * mSbcs[i]; }
  Sophus::SE3d&      getSbc(int i) { return mSbcs[i]; }
  MapPointFactorMap& getMapPointFactorMap() { return mMapPointFactorMap; }
};

}  //namespace db
}  //namespace toy