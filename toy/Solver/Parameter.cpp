#include <algorithm>
#include <limits>
#include <sophus/so3.hpp>
#include "Frame.h"
#include "MapPoint.h"
#include "Parameter.h"

namespace toy {
FrameParameter::FrameParameter(db::Frame::Ptr frame) {
  mId = frame->id();

  mTwb       = frame->getTwb();
  mBackupTwb = mTwb;

  mDel.setZero();
}
void FrameParameter::backup() {
  mBackupTwb = mTwb;
  mBackupDel = mDel;
}

void FrameParameter::update(const Eigen::Vector6d& delta) {
  auto& Pwb = mTwb.translation();
  Pwb += delta.head(3);

  auto& so3wb = mTwb.so3();
  so3wb *= Sophus::SO3d::exp(delta.tail(3));
}

MapPointParameter::MapPointParameter(db::MapPoint::Ptr mp) {
  mId = mp->id();

  mUndist       = mp->undist();
  mBackupUndist = mUndist;

  mInvD       = mp->invDepth();
  mBackupInvD = mInvD;
}

void MapPointParameter::backup() {
  mBackupUndist = mUndist;
  mBackupInvD   = mInvD;
}

void MapPointParameter::update(const Eigen::Vector3d& delta) {
  mUndist += delta.head(2);
  mInvD = std::max(1e-5, mInvD + delta.z());
}
}  //namespace toy