#include "ToyLogger.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "LocalMap.h"
#include "LocalTracker.h"
#include "BasicSolver.h"
#include "VioSolver.h"

namespace toy {
LocalTracker::LocalTracker()
  : mStatus{Status::NONE}
  , mLocalMap{nullptr} {}

LocalTracker::~LocalTracker() {
  mLocalMap.release();
  mVioSolver.release();
}

void LocalTracker::prepare() {
  mLocalMap = std::make_unique<db::LocalMap>();
  mStatus   = Status::INITIALIZING;
}

void LocalTracker::process() {
  db::Frame::Ptr currFrame = getLatestInput();
  if (!currFrame)
    return;

  switch (mStatus) {
  case Status::NONE: {
    ToyLogE("prepare tracker before process");
    break;
  }
  case Status::INITIALIZING: {
    mLocalMap->addFrame(currFrame);
    int  createMPCount = initializeMapPoints(currFrame);
    bool OK            = createMPCount > Config::Vio::initializeMapPointCount;
    if (OK) {
      mStatus = Status::TRACKING;
    }
    else {
      mLocalMap->reset();
    }
    break;
  }
  case Status::TRACKING: {

    //todo changed if imu exists;
    if (true) {
      auto& Swb = mLocalMap->getFrames().rbegin()->second->getSwb();
      currFrame->setSwb(Swb);
    }

    mLocalMap->addFrame(currFrame);
    int createMPCount = initializeMapPoints(currFrame);

    BasicSolver::solveFramePose(currFrame);

    std::vector<db::Frame::Ptr>    frames;
    std::vector<db::MapPoint::Ptr> mapPoints;

    mLocalMap->getCurrentStates(frames, mapPoints);
    mVioSolver->solve(frames, mapPoints);

    break;
  }
  }
}

int LocalTracker::initializeMapPoints(db::Frame::Ptr currFrame) {
  auto& mapPointFactorMap = currFrame->getMapPointFactorMap();

  auto            Swc0  = currFrame->getSwc(0);
  auto            Swc1  = currFrame->getSwc(1);  //identity for mono or depth..
  auto            Sc1c0 = Swc1.inverse() * Swc0;
  Eigen::Vector3d Pc0x;

  int successCount = 0;

  for (auto& [mpWeak, factor] : mapPointFactorMap) {
    auto mpPtr = mpWeak.lock();
    if (mpPtr->status() != db::MapPoint::Status::INITIALING)
      continue;

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
      mpPtr->setState(db::MapPoint::Status::TRACKING);
      ++successCount;
      break;
    }
    case db::ReprojectionFactor::Type::DEPTH: {
      break;
    }
    }
  }
  return successCount;
}
}  //namespace toy