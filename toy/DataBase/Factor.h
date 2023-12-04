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
class ReprojectionFactor {
public:
  ReprojectionFactor()  = delete;
  ~ReprojectionFactor() = default;
  ReprojectionFactor(Frame*          frame,
                     MapPoint*       mapPoint,
                     Eigen::Vector2d uv0,
                     Eigen::Vector3d unidst0,
                     Eigen::Vector2d uv1     = Eigen::Vector2d(0.0, -1.0),
                     Eigen::Vector3d undist1 = Eigen::Vector3d(0.0, 0.0, -1.0))
    : mType{0}
    , mFrame{frame}
    , mMapPoint{mapPoint}
    , mUV0{uv0}
    , mUndist0{unidst0}
    , mUV1{uv1}
    , mUndist1{undist1} {}

  void addStereo(Eigen::Vector2d uv1, Eigen::Vector3d undist1) {
    mType    = 1;
    mUV1     = uv1;
    mUndist1 = undist1;
  }

protected:
  int             mType;
  Frame*          mFrame;
  MapPoint*       mMapPoint;
  Eigen::Vector2d mUV0;
  Eigen::Vector3d mUndist0;
  Eigen::Vector2d mUV1;
  Eigen::Vector3d mUndist1;

public:
  const int getType() const { return mType; }
};

}  //namespace db
}  //namespace toy