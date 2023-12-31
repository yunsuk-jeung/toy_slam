#include <sophus/se3.hpp>
#include "ToyLogger.h"
#include "Frame.h"
#include "MapPoint.h"
namespace toy {
namespace db {

MapPoint::MapPoint(int id, int hostFrameId)
  : mId{id}
  , mHostFrameId{hostFrameId}
  , mStatus{Status::INITIALING}
  , mInvDepth{1.0}
  , mBackupInvD{1.0}
  , mFixed{false} {
  mFrameFactors.reserve(50);
}

void MapPoint::addFrameFactor(std::shared_ptr<db::Frame> frame,
                              ReprojectionFactor         factor) {
  mFrameFactors.push_back({frame, factor});
}

void MapPoint::backup() {
  mBackupUndist = mUndist;
  mBackupInvD   = mInvDepth;
}

void MapPoint::restore() {
  mUndist   = mBackupUndist;
  mInvDepth = mBackupInvD;
}

void MapPoint::update(const Eigen::Vector3d& delta) {
  mUndist += delta.head(2);
  mInvDepth = std::max(1e-5, mInvDepth + delta.z());
}

void MapPoint::update(const double& delta) {
  mInvDepth = std::max(1e-5, mInvDepth + delta);
}

Eigen::Vector3d MapPoint::getPwx() {
  auto& [frameW, factor] = mFrameFactors.front();
  auto fPtr              = frameW.lock();

  Eigen::Vector3d Pc0x = Eigen::Vector3d(mUndist.x(), mUndist.y(), 1.0) / mInvDepth;
  return fPtr->getTwc(0) * Pc0x;
}

}  //namespace db
}  //namespace toy
