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
}  //namespace toy