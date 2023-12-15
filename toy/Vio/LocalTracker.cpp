#include "ToyLogger.h"
#include "SLAMInfo.h"
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
  mLocalMap.reset();
  mVioSolver.reset();
}

void LocalTracker::prepare() {
  mLocalMap  = std::make_unique<db::LocalMap>();
  mVioSolver = VioSolverFactory::createVioSolver();

  mStatus = Status::INITIALIZING;
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
    currFrame->setKeyFrame();

    if (OK) {
      mStatus = Status::TRACKING;
    }
    else {
      mLocalMap->reset();
      return;
    }
    break;
  }
  case Status::TRACKING: {
    //todo changed if imu exists;
    if (true) {
      auto& Twb = mLocalMap->getFrames().rbegin()->second->getTwb();
      currFrame->setTwb(Twb);
    }

    if (mLocalMap->addFrame(currFrame)) {
      currFrame->setKeyFrame();
    }

    int createMPCount = initializeMapPoints(currFrame);

    BasicSolver::solveFramePose(currFrame);

    std::vector<db::Frame::Ptr>    frames;
    std::vector<db::MapPoint::Ptr> mapPoints;

    mLocalMap->getCurrentStates(frames, mapPoints);
    mVioSolver->solve(frames, mapPoints);

    int id = getMarginalFrameId(frames);

    if (id < 0) {
      break;
    }

    mVioSolver->marginalize(id);
    mLocalMap->removeFrame(id);

    break;
  }
  }

  setDataToInfo();
}

int LocalTracker::initializeMapPoints(db::Frame::Ptr currFrame) {
  auto& mapPointFactorMap = currFrame->getMapPointFactorMap();

  auto            Twc0  = currFrame->getTwc(0);
  auto            Twc1  = currFrame->getTwc(1);  //identity for mono or depth..
  auto            Tc1c0 = Twc1.inverse() * Twc0;
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
      if (!BasicSolver::triangulate(factor.undist0(), factor.undist1(), Tc1c0, Pc0x)) {
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

int LocalTracker::getMarginalFrameId(std::vector<db::Frame::Ptr>& frames) {
  ToyLogE("NOT IMPLEMENTED YET");
  return 0;
}

void LocalTracker::setDataToInfo() {
  auto* info     = SLAMInfo::getInstance();
  auto& frameMap = mLocalMap->getFrames();

  std::vector<Eigen::Matrix4f> Mwcs;
  Mwcs.reserve(frameMap.size());

  for (auto& [key, framePtr] : frameMap) {
    Eigen::Matrix4f Mwc = framePtr->getTwc(0).matrix().cast<float>();
    Mwcs.push_back(std::move(Mwc));
  }
  info->setLocalPath(Mwcs);

  auto& mpMap  = mLocalMap->getMapPoints();
  auto  mpSize = mpMap.size();

  std::vector<float> outmp;
  outmp.reserve(mpSize * 4);

  for (auto& [key, mpPtr] : mpMap) {
    if (mpPtr->status() < db::MapPoint::Status::TRACKING)
      continue;

    Eigen::Vector3d Pwx = mpPtr->getPwx();
    outmp.push_back(Pwx.x());
    outmp.push_back(Pwx.y());
    outmp.push_back(Pwx.z());
    outmp.push_back(1.0);
  }

  info->setLocalPoints(outmp);
}

}  //namespace toy