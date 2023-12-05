#include "ToyLogger.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "LocalMap.h"
#include "LocalTracker.h"
#include "BasicSolver.h"

namespace toy {
LocalTracker::LocalTracker()
  : mStatus{Status::NONE}
  , mLocalMap{nullptr} {}

LocalTracker::~LocalTracker() {
  delete mLocalMap;
  mLocalMap = nullptr;
}

void LocalTracker::prepare() {
  mLocalMap = new db::LocalMap();
  mStatus   = Status::INITIALIZING;
}

void LocalTracker::process() {
  ToyLogD("localTracker queue size : {}", mInQueue.unsafe_size());

  db::Frame::Ptr currFrame = getLatestInput();

  switch (mStatus) {
  case Status::NONE: {
    ToyLogE("prepare tracker before process");
    break;
  }
  case Status::INITIALIZING: {
    bool OK = initialize(currFrame);
    if (OK) { mStatus = Status::TRACKING; }
    else { mLocalMap->reset(); }
    break;
  }
  case Status::TRACKING: {
    break;
  }
  }
}

bool LocalTracker::initialize(db::Frame::Ptr currFrame) {
  mLocalMap->addFrame(currFrame);
  auto& mapPointFactorMap = currFrame->getMapPointFactorMap();

  auto            Swc0  = currFrame->getSwc(0);
  auto            Swc1  = currFrame->getSwc(1);  //identity for mono or depth..
  auto            Sc1c0 = Swc1.inverse() * Swc0;
  Eigen::Vector3d Pc0x;

  int successCount = 0;

  for (auto& [mpWeak, factor] : mapPointFactorMap) {
    auto mpPtr = mpWeak.lock();
    if (mpPtr->status() != db::MapPoint::Status::INITIALING) continue;
    switch (factor.getType()) {
    case db::ReprojectionFactor::Type::MONO: {
      break;
    }
    case db::ReprojectionFactor::Type::STEREO: {
      if (!BasicSolver::triangulate(factor.undist0(), factor.undist1(), Sc1c0, Pc0x)) {
        continue;
      }
      double invD = 1.0 / Pc0x.z();
      mpPtr->setInvDepth(invD);
      ++successCount;
      break;
    }
    case db::ReprojectionFactor::Type::DEPTH: {
      break;
    }
    }
  }
  if (successCount < Config::Vio::initializeMapPointCount) return false;

  return true;
}

}  //namespace toy