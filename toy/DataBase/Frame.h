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
class CameraInfo;
namespace db {
class ImagePyramid;
class ImagePyramidSet;
class LocalMap;
class Feature;
class MapPoint;
class FrameState;
class Frame {
public:
  USING_SMART_PTR(Frame);
  DELETE_COPY_CONSTRUCTORS(Frame);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame();
  Frame(FrameState*                       frameState,
        size_t                            cameraId,
        std::shared_ptr<db::ImagePyramid> imagePyramid,
        std::unique_ptr<Camera>           camera);
  ~Frame();

  Frame::Ptr clone(FrameState* fs);

  void setTbc(float*);

  void addMapPointFactor(std::shared_ptr<db::MapPoint> mp, ReprojectionFactor factor);
  void drawReprojectionView(std::string imshowName = "reproj view", bool half = false);

protected:

protected:
  using MapPointFactorMap = std::map<std::weak_ptr<db::MapPoint>,
                                     ReprojectionFactor,
                                     std::owner_less<std::weak_ptr<db::MapPoint>>>;

  size_t                            mCameraId;
  std::shared_ptr<db::ImagePyramid> mImagePyramid;
  std::unique_ptr<Camera>           mCamera;
  std::unique_ptr<Feature>          mFeature;

  MapPointFactorMap mMapPointFactorMap;

  //S : se3
  FrameState*   mFrameState;
  Sophus::SE3d& mTwb;
  Sophus::SE3d  mTbc;

public:
  const size_t       cameraId() const { return mCameraId; }
  ImagePyramid*      getImagePyramid() { return mImagePyramid.get(); }
  Camera*            getCamera() { return mCamera.get(); }
  Feature*           getFeature() { return mFeature.get(); }
  Sophus::SE3d       getTwc() { return mTwb * mTbc; }
  Sophus::SE3d&      getTbc() { return mTbc; }
  MapPointFactorMap& getMapPointFactorMap() { return mMapPointFactorMap; }
};

}  //namespace db
}  //namespace toy