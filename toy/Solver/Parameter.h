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

  void backup();

  int          mId;
  Sophus::SE3d mTwb;
  Sophus::SE3d mBackupTwb;

  Eigen::Vector6d      mDel;  //for marginalize
  Eigen::Vector6d      mBackupDel;
  static constexpr int SIZE = 6;
};

class MapPointParameter {
public:
  MapPointParameter() = default;
  MapPointParameter(std::shared_ptr<db::MapPoint> mp);
  ~MapPointParameter() = default;

  void backup();

  int             mId;
  Eigen::Vector2d mUndist;  //normalized uv
  double          mInvD;    //invDepth

  Eigen::Vector2d mBackupUndist;
  double          mBackupInvD;

  static constexpr int SIZE = 3;
};
}  //namespace toy