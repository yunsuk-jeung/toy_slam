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
                     Eigen::Vector2d uv1     = Eigen::Vector2d(-1.0, -1.0),
                     Eigen::Vector3d undist1 = Eigen::Vector3d(-1.0, -1.0, -1.0))
    : mType{Type::MONO}
    , mFrame{frame}
    , mMapPoint{mapPoint}
    , mUV0{uv0}
    , mUndist0{unidst0}
    , mUV1{uv1}
    , mUndist1{undist1} {}

  void addStereo(Eigen::Vector2d uv1, Eigen::Vector3d undist1) {
    mType    = Type::STEREO;
    mUV1     = uv1;
    mUndist1 = undist1;
  }

public:
  enum class Type { MONO = 0, STEREO = 1, DEPTH = 2 };

protected:
  Type            mType;
  Frame*          mFrame;
  MapPoint*       mMapPoint;
  Eigen::Vector2d mUV0;
  Eigen::Vector3d mUndist0;
  Eigen::Vector2d mUV1;
  Eigen::Vector3d mUndist1;

public:
  const Type       getType() const { return mType; }
  Eigen::Vector2d& uv0() { return mUV0; }
  Eigen::Vector3d& undist0() { return mUndist0; }
  Eigen::Vector2d& uv1() { return mUV1; }
  Eigen::Vector3d& undist1() { return mUndist1; }
};

}  //namespace db
}  //namespace toy