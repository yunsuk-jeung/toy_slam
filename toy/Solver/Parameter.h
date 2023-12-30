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
  void restore();
  void update(const Eigen::Vector6d& delta);

protected:
  int          mId;
  Sophus::SE3d mTwb;
  Sophus::SE3d mBackupTwb;

  Eigen::Vector6d mDel;  //for marginalize
  Eigen::Vector6d mBackupDel;

  std::shared_ptr<db::Frame> mFrame;

public:
  const int&          id() const { return mId; }
  const Sophus::SE3d& Twb() const { return mTwb; }

  static constexpr size_t SIZE = 6;
};

class MapPointParameter {
public:
  MapPointParameter() = default;
  MapPointParameter(std::shared_ptr<db::MapPoint> mp);
  ~MapPointParameter() = default;

  void backup();
  void restore();
  void update(const Eigen::Vector3d& delta);

protected:
  int             mId;
  Eigen::Vector2d mUndist;  //normalized uv
  double          mInvD;    //invDepth

  Eigen::Vector2d               mBackupUndist;
  double                        mBackupInvD;

  std::shared_ptr<db::MapPoint> mMapPoint;

public:
  const int&             id() const { return mId; }
  const Eigen::Vector2d& undist() const { return mUndist; }
  const double&          invD() const { return mInvD; }

  static constexpr size_t SIZE = 3;
};
}  //namespace toy