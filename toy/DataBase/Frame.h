#pragma once
#include <array>
#include <memory>
#include <map>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "CustomTypes.h"
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
  static constexpr size_t PARAMETER_SIZE = 6;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame(std::shared_ptr<ImagePyramidSet> set);
  ~Frame();
  Frame(Frame* src);

  Frame::Ptr clonePtr();

  void setCameras(Camera* cam0, Camera* cam1);
  void setTbc(float*, float*);

  void addMapPointFactor(std::shared_ptr<db::MapPoint> mp, ReprojectionFactor factor);
  void eraseMapPointFactor(size_t mapPointId);

  void resetDelta();
  void backup();
  void restore();
  void update(const Eigen::Vector6d& delta);

  void drawReprojectionView(int         idx,
                            std::string imshowName = "reproj view",
                            bool        half       = false);

  Eigen::Vector<double, PARAMETER_SIZE> toParameter() const {
    Eigen::Vector<double, PARAMETER_SIZE> out;
    out.head(3) = this->Twb().translation();
    out.tail(3) = this->Twb().so3().log();
    return out;
  }

protected:
  using MapPointFactorMap = std::map<int64_t, ReprojectionFactor>;

  static int64_t globalId;
  int64_t        mId;
  bool           mIsKeyFrame;

  std::array<std::shared_ptr<db::ImagePyramid>, 2> mImagePyramids;
  std::array<std::unique_ptr<Camera>, 2>           mCameras;
  std::array<std::unique_ptr<Feature>, 2>          mFeatures;

  std::vector<MapPointFactorMap> mMapPointFactorMaps;
  //S : se3
  std::array<Sophus::SE3d, 2> mTbcs;

  Sophus::SE3d mTwb;
  Sophus::SE3d mBackupTwb;

  Eigen::Vector6d mDelta;  //for marginalize
  Eigen::Vector6d mBackupDelta;

  bool mFixed;
  bool mLinearized;

public:
  const int64_t       id() const { return mId; }
  void                setKeyFrame() { mIsKeyFrame = true; }
  const bool          isKeyFrame() const { return mIsKeyFrame; }
  ImagePyramid*       getImagePyramid(size_t i) { return mImagePyramids[i].get(); }
  Camera*             getCamera(size_t i) { return mCameras[i].get(); }
  Feature*            getFeature(size_t i) { return mFeatures[i].get(); }
  auto&               getFeatures() { return mFeatures; }
  void                setTwb(const Sophus::SE3d& Swb) { mTwb = Swb; }
  const Sophus::SE3d& Twb() const { return mTwb; }
  Sophus::SE3d&       getTwb() { return mTwb; }
  Sophus::SE3d        getTwc(size_t i) { return mTwb * mTbcs[i]; }
  Sophus::SE3d&       getTbc(size_t i) { return mTbcs[i]; }
  std::vector<MapPointFactorMap>& mapPointFactorMaps() { return mMapPointFactorMaps; }
  MapPointFactorMap& mapPointFactorMap(size_t i) { return mMapPointFactorMaps[i]; }
  const bool         fixed() const { return mFixed; }
  void               setFixed(bool fixed) { mFixed = fixed; }
  void               setLinearized(bool linearized) { mLinearized = linearized; }
  bool               isLinearized() { return mLinearized; }
  Eigen::Vector6d    getDelta() { return mDelta; };
};

}  //namespace db
}  //namespace toy