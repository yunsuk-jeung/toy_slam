#pragma once

#include <sophus/se3.hpp>
#include "usings.h"
namespace toy {
namespace db {
class Frame;
class MapPoint;
}  //namespace db
class FrameParameter {
public:
  FrameParameter() = default;
  FrameParameter(std::shared_ptr<db::Frame> frame);
  ~FrameParameter() = default;

  Sophus::SE3d         mTwb;
  Eigen::Vector6d      mDel;  //for marginalize
  static constexpr int SIZE = 6;
};

class MapPointParameter {
public:
  MapPointParameter() = default;
  MapPointParameter(std::shared_ptr<db::MapPoint> mp);
  ~MapPointParameter() = default;

  Eigen::Vector2d      mUndist;  //normalized uv
  double               mInvD;    //invDepth
  static constexpr int SIZE = 3;
};
}  //namespace toy