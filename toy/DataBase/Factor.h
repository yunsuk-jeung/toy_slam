#pragma once
#include <Eigen/Dense>

namespace toy {
namespace db {
class Frame;
class MapPoint;

class Factor {
public:
  Factor(Frame* frame, MapPoint* mp)
    : mFrame{frame}
    , mMapPoint{mp} {}
  ~Factor() {
    mFrame    = nullptr;
    mMapPoint = nullptr;
  }

protected:
  Frame*    mFrame;
  MapPoint* mMapPoint;

public:
  const Frame*    frame() const { return mFrame; }
  const MapPoint* mapPoint() const { return mMapPoint; }
};

class ReprojectionFactor : public Factor {
public:
  ReprojectionFactor()  = delete;
  ~ReprojectionFactor() = default;
  ReprojectionFactor(Frame*          frame,
                     MapPoint*       mapPoint,
                     Eigen::Vector2d uv,
                     Eigen::Vector3d undist)
    : Factor(frame, mapPoint)
    , mUV{uv}
    , mUndist{undist} {}

public:

protected:
  Eigen::Vector2d mUV;
  Eigen::Vector3d mUndist;

public:
  Eigen::Vector2d& uv() { return mUV; }
  Eigen::Vector3d& undist() { return mUndist; }
};

}  //namespace db
}  //namespace toy