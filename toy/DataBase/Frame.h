#pragma once
#include <array>
#include <memory>
#include <map>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "usings.h"
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

  void resetDelta();
  void backup();
  void restore();
  void update(const Eigen::Vector6d& delta);

  void drawReprojectionView(int idx, std::string imshowName = "reproj view");

protected:

protected:
  using MapPointFactorMap = std::map<std::weak_ptr<db::MapPoint>,
                                     ReprojectionFactor,
                                     std::owner_less<std::weak_ptr<db::MapPoint>>>;
  static int globalId;
  int        mId;
  bool       mIsKeyFrame;

  std::array<std::unique_ptr<ImagePyramid>, 2> mImagePyramids;
  std::array<std::unique_ptr<Camera>, 2>       mCameras;
  std::array<std::unique_ptr<Feature>, 2>      mFeatures;

  MapPointFactorMap mMapPointFactorMap;

  ////L : lie
  //Eigen::Vector6d                mLwb;
  //std::array<Eigen::Vector6d, 2> mLbcs;
  //Eigen::Vector6d&   getLbc(int i) { return mLbcs[i]; }

  //S : se3
  std::array<Sophus::SE3d, 2> mTbcs;

  Sophus::SE3d mTwb;
  Sophus::SE3d mBackupTwb;

  Eigen::Vector6d mDelta;  //for marginalize
  Eigen::Vector6d mBackupDelta;

  bool mFixed;

public:
  const int           id() const { return mId; }
  void                setKeyFrame() { mIsKeyFrame = true; }
  const bool          isKeyFrame() const { return mIsKeyFrame; }
  ImagePyramid*       getImagePyramid(int i) { return mImagePyramids[i].get(); }
  Camera*             getCamera(int i) { return mCameras[i].get(); }
  Feature*            getFeature(int i) { return mFeatures[i].get(); }
  void                setTwb(const Sophus::SE3d& Swb) { mTwb = Swb; }
  const Sophus::SE3d& Twb() const { return mTwb; }
  Sophus::SE3d&       getTwb() { return mTwb; }
  Sophus::SE3d        getTwc(int i) { return mTwb * mTbcs[i]; }
  Sophus::SE3d&       getTbc(int i) { return mTbcs[i]; }
  MapPointFactorMap&  getMapPointFactorMap() { return mMapPointFactorMap; }
  const bool          fixed() const { return mFixed; }
  void                setFixed(bool fixed) { mFixed = fixed; }

  static constexpr size_t PARAMETER_SIZE = 6;
};

}  //namespace db
}  //namespace toy