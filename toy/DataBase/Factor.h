#pragma once
#include <Eigen/Dense>

namespace toy {
namespace db {
class Frame;
class MapPoint;

//class Factor {
//public:
//
//protected:
//}

/**
 * @brief mappoint and frame factor
 */

class Factor {
public:
  Factor() = delete;
  virtual ~Factor() {
    mFrame    = nullptr;
    mMapPoint = nullptr;
  }

  Factor(std::shared_ptr<Frame> frame, size_t camId, std::shared_ptr<MapPoint> mapPoint)
    : mFrame{frame}
    , mCamId{camId}
    , mMapPoint{mapPoint} {}

  enum class Type { NONE = 0, REPROJECTION = 1, DEPTH = 2 };

protected:
  virtual void              setType() = 0;
  Type                      mType;
  std::shared_ptr<Frame>    mFrame;
  size_t                    mCamId;
  std::shared_ptr<MapPoint> mMapPoint;

public:
  auto&                     type() { return mType; }
  std::shared_ptr<Frame>    frame() { return mFrame; }
  std::shared_ptr<MapPoint> mapPoint() { return mMapPoint; }
};

//class ReprojectionFactor {
//public:
//  ReprojectionFactor()  = delete;
//  ~ReprojectionFactor() = default;
//  ReprojectionFactor(Frame*          frame,
//                     MapPoint*       mapPoint,
//                     Eigen::Vector2d uv0,
//                     Eigen::Vector3d unidst0,
//                     Eigen::Vector2d uv1     = Eigen::Vector2d(-1.0, -1.0),
//                     Eigen::Vector3d undist1 = Eigen::Vector3d(-1.0, -1.0, -1.0))
//    : mType{Type::MONO}
//    , mFrame{frame}
//    , mMapPoint{mapPoint}
//    , mUV0{uv0}
//    , mUndist0{unidst0}
//    , mUV1{uv1}
//    , mUndist1{undist1} {}
//
//  void addStereo(Eigen::Vector2d uv1, Eigen::Vector3d undist1) {
//    mType    = Type::STEREO;
//    mUV1     = uv1;
//    mUndist1 = undist1;
//  }
//
//public:

//
//protected:
//  Frame*          mFrame;
//  MapPoint*       mMapPoint;
//  Eigen::Vector2d mUV0;
//  Eigen::Vector3d mUndist0;
//  Eigen::Vector2d mUV1;
//  Eigen::Vector3d mUndist1;
//
//public:
//  const Type       type() const { return mType; }
//  Eigen::Vector2d& uv0() { return mUV0; }
//  Eigen::Vector3d& undist0() { return mUndist0; }
//  Eigen::Vector2d& uv1() { return mUV1; }
//  Eigen::Vector3d& undist1() { return mUndist1; }
//};

class ReprojectionFactor : public Factor {
public:
  ReprojectionFactor()  = delete;
  ~ReprojectionFactor() = default;
  ReprojectionFactor(std::shared_ptr<Frame>    frame,
                     size_t                    camId,
                     std::shared_ptr<MapPoint> mapPoint,
                     Eigen::Vector2d           uv,
                     Eigen::Vector3d           unidst)
    : Factor{frame, camId, mapPoint}
    , mUV{uv}
    , mUndist{unidst} {
    setType();
  }

protected:
  void setType() override { mType = Type::REPROJECTION; }

  Eigen::Vector2d mUV;
  Eigen::Vector3d mUndist;

public:
  Eigen::Vector2d& uv() { return mUV; }
  Eigen::Vector3d& undist() { return mUndist; }
  size_t&          camIdx() { return mCamId; }
};

}  //namespace db
}  //namespace toy