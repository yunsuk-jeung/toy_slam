#include "Frame.h"
#include "MapPoint.h"

#include "Parameter.h"

namespace toy {
FrameParameter::FrameParameter(db::Frame::Ptr frame) {
  mTwb = frame->getTwb();
  mDel.setZero();
}

MapPointParameter::MapPointParameter(db::MapPoint::Ptr mp) {
  mUndist = mp->undist();
  mInvD   = mp->invDepth();
}
}  //namespace toy