#include <sophus/se3.hpp>
#include "Frame.h"
#include "MapPoint.h"
namespace toy {
namespace db {

MapPoint::MapPoint(int id, int hostFrameId)
  : mId{id}
  , mHostFrameId{hostFrameId}
  , mStatus{Status::INITIALING}
  , mInvDepth{1.0} {
  mFrameFactors.reserve(50);
}

void MapPoint::addFrameFactor(std::shared_ptr<db::Frame> frame,
                              ReprojectionFactor         factor) {
  mFrameFactors.push_back({frame, factor});
}

Eigen::Vector3d MapPoint::getPwx() {
  auto& [frameW, factor] = mFrameFactors.front();
  auto fPtr              = frameW.lock();

  Eigen::Vector3d undist(mUndist.x(), mUndist.y(), 1.0);

  return fPtr->getTwc(0) * undist / mInvDepth;
}

}  //namespace db
}  //namespace toy
