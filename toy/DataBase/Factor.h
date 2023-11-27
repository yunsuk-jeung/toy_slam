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
class MFFactor {
public:
  MFFactor(Frame* frame, MapPoint* mapPoint, float nu, float, nv);

protected:
  Frame*          mFrame;
  MapPoint*       mMapPoint;
  Eigen::Vector3d mUndist;
}

}  //namespace db
}  //namespace toy