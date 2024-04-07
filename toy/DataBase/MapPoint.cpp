#include <sophus/se3.hpp>
#include "ToyAssert.h"
#include "Frame.h"
#include "MapPoint.h"
namespace toy {
namespace db {

MapPoint::MapPoint(size_t id)
  : mId{id}  //, mHostFrameId{-1}
  , mStatus{Status::NONE}
  , mInvDepth{1.0}
  , mBackupInvD{1.0}
  , mFixed{false} {
  mFrameFactorMap.reserve(10);
  mMarginedPwx.setZero();
}

//void MapPoint::setHost(std::shared_ptr<db::Frame> host) {
//
//}

void MapPoint::setHost(std::shared_ptr<db::Frame> host) {
  mHostFrame = host;
}

void MapPoint::addFrameFactor(std::shared_ptr<db::Frame> frame,
                              ReprojectionFactor         factor) {
  FrameCamId key{frame->id(), factor.camIdx()};
  mFrameFactorMap.insert({key, factor});
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

bool MapPoint::eraseFrame(std::shared_ptr<db::Frame> frame) {
  TOY_ASSERT(!mFrameFactorMap.empty());

  bool REMOVE_THIS = false;

  size_t camSize = frame->getFeatures().size();
  if (frame != mHostFrame) {
    this->mStatus    = Status::NONE;
    bool REMOVE_THIS = true;
  }

  for (size_t i = 0; i < camSize; ++i) {
    mFrameFactorMap.erase({frame->id(), i});
  }

  mHostFrame = nullptr;

  return !REMOVE_THIS;
}

Eigen::Vector3d MapPoint::getPwx() {
  Eigen::Vector3d Pc0x = Eigen::Vector3d(mUndist.x(), mUndist.y(), 1.0) / mInvDepth;
  return mHostFrame->getTwc(0) * Pc0x;
}

}  //namespace db
}  //namespace toy
