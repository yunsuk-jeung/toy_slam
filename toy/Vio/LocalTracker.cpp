#include "ToyLogger.h"
#include "SLAMInfo.h"
#include "Feature.h"
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
  , mLocalMap{nullptr}
  , mKeyFrameInterval{0} {}

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
    ++mKeyFrameInterval;

    //YSTODO: changed if imu exists;
    if (true) {
      auto& Twb = mLocalMap->getFrames().rbegin()->second->getTwb();

      currFrame->setTwb(Twb);
    }

    if (mLocalMap->addFrame(currFrame)) {
      currFrame->setKeyFrame();
      mKeyFrameInterval = 0;

      int createMPCount = initializeMapPoints(currFrame);
    }

    //if (!currFrame->isKeyFrame()) {
    //auto& mapPointMap = mLocalMap->getMapPoints();
    //auto& ids         = currFrame->getFeature(0)->getKeypoints().mIds;
    //int   count       = 0;

    //for (auto& id : ids) {
    //  if (mapPointMap.count(id))
    //    count++;
    //}

    //float ratio = float(count) / mapPointMap.size();

    //if (float(count) / mapPointMap.size() < Config::Vio::newKeframeFeatureRatio
    //    && mKeyFrameInterval > Config::Vio::minKeyFrameCount) {
    //  currFrame->setKeyFrame();
    //  mKeyFrameInterval = 0;
    //}
    //}

    //YSTODO: check quality with createdMPCount
    std::vector<db::Frame::Ptr>    frames;
    std::vector<db::MapPoint::Ptr> mapPoints;
    mLocalMap->getCurrentStates(frames, mapPoints);

    //drawDebugView(100, 0);

    //BasicSolver::solveFramePose(currFrame);

    frames.front()->setFixed(true);
    mVioSolver->solve(frames, mapPoints);

    //drawDebugView(101, 1040);
    drawDebugView(101, 1040);

    //cv::waitKey();

    db::Frame::Ptr marginalFrame = selectMarginalFrame(frames);

    if (!marginalFrame) {
      break;
    }

    mVioSolver->marginalize(marginalFrame);
    mLocalMap->removeFrame(marginalFrame->id());

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
    auto mp = mpWeak.lock();
    if (mp->status() != db::MapPoint::Status::INITIALING)
      continue;

    switch (factor.getType()) {
    case db::ReprojectionFactor::Type::MONO: {
      break;
    }
    case db::ReprojectionFactor::Type::STEREO: {
      if (!BasicSolver::triangulate(factor.undist0(), factor.undist1(), Tc1c0, Pc0x)) {
        continue;
      }

      double          invD = 1.0 / Pc0x.z();
      Eigen::Vector2d nuv  = Pc0x.head(2) * invD;
      mp->setUndist(nuv);
      mp->setInvDepth(invD);
      mp->setState(db::MapPoint::Status::TRACKING);
      ++successCount;
      break;
    }
    case db::ReprojectionFactor::Type::DEPTH: {
      break;
    }
    }
  }

  if (Config::Vio::debug) {
    ToyLogD("triangulation successed {} / {}", successCount, mapPointFactorMap.size());
  }

  if (Config::Vio::showStereoTracking) {
    currFrame->drawReprojectionView(0, "tri0");
    currFrame->drawReprojectionView(1, "tri1");
    cv::waitKey();
  }

  return successCount;
}

db::Frame::Ptr LocalTracker::selectMarginalFrame(std::vector<db::Frame::Ptr>& allFrames) {
  std::vector<db::Frame::Ptr> keyFrames;
  std::vector<db::Frame::Ptr> frames;
  keyFrames.reserve(Config::Vio::maxKeyFrameSize);
  frames.reserve(Config::Vio::maxKeyFrameSize);

  for (auto& frame : allFrames) {
    if (frame->isKeyFrame()) {
      keyFrames.push_back(frame);
    }
    else {
      frames.push_back(frame);
    }
  }

  if (frames.size() > Config::Vio::maxFrameSize) {
    return frames.front();
  }

  if (keyFrames.size() > Config::Vio::maxKeyFrameSize) {
    return keyFrames.front();
  }

  return nullptr;

  std::vector<db::Frame::Ptr> cands;
  cands.reserve(3);
  cands.push_back(keyFrames.front());
  cands.push_back(frames.front());

  auto secondLastFrameIt = std::prev(allFrames.end(), 2);

  auto& secondLast     = *secondLastFrameIt;
  auto& prevSecondLast = *(--secondLastFrameIt);

  if (!secondLast->isKeyFrame()) {
    auto& lastKeyFrame = keyFrames.back();

    auto& mpFactorMap0 = prevSecondLast->getMapPointFactorMap();
    auto& mpFactorMap1 = secondLast->getMapPointFactorMap();

    int    count      = 0;
    double parallaxSq = 0;

    auto endIt = mpFactorMap1.end();
    for (auto& [key, val] : mpFactorMap0) {
      auto targetIt = mpFactorMap1.find(key);
      if (targetIt == endIt) {
        continue;
      }
      ++count;
      parallaxSq += (val.uv0() - targetIt->second.uv0()).squaredNorm();
    }

    if (parallaxSq / count < Config::Vio::minParallaxSqNorm) {
      return secondLast;
    }
  }
  //else {
  return frames.front();
  //}

  auto& keyFrame = keyFrames.back();

  auto& mpFactorMap0 = keyFrame->getMapPointFactorMap();
  auto& mpFactorMap1 = secondLast->getMapPointFactorMap();

  int  count = 0;
  auto endIt = mpFactorMap1.end();
  for (auto& [key, val] : mpFactorMap0) {
    if (mpFactorMap1.find(key) == endIt) {
      continue;
    }
    ++count;
  }

  if (count / mpFactorMap0.size() < (1.0 - Config::Vio::minTrackedRatio)) {
    return keyFrame;
  }
  else {
    return frames.front();
  }

  return nullptr;
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

void LocalTracker::drawDebugView(int tag, int offset) {
  int                            id = 0;
  std::vector<db::Frame::Ptr>    frames;
  std::vector<db::MapPoint::Ptr> mapPoints;
  mLocalMap->getCurrentStates(frames, mapPoints);

  if (frames.empty()) {
    return;
  }

  int xOffset = frames.front()->getImagePyramid(0)->getOrigin().cols;
  int yOffset = frames.front()->getImagePyramid(0)->getOrigin().rows;

  int k = 0;
  for (auto f : frames) {
    std::string name = std::to_string(tag) + "_" + std::to_string(k++);
    f->drawReprojectionView(0, name, false);

    if (id < 5)
      cv::moveWindow(name, (xOffset * id), (offset));
    else
      cv::moveWindow(name, (xOffset * (id - 5)), (offset + 40 + yOffset));
    ++id;
  }
}
}  //namespace toy