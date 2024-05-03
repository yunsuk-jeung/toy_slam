#include <set>
#include "ToyAssert.h"
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
namespace {
constexpr bool NO_IMU = true;
}
LocalTracker::LocalTracker()
  : mStatus{Status::NONE}
  , mLocalMap{nullptr}
  , mKeyFrameAfter{0}
  , mSetKeyFrame{false} {
  mMarginalFrameIds.reserve(Config::Vio::maxKeyFrameSize);
  TAG = "LocalTracker";
}

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

  if (Config::Vio::debug) {
    ToyLogD("-------------- {} frame {:4d} --------------", TAG, currFrame->id());
  }

  switch (mStatus) {
  case Status::NONE: {
    ToyLogE("prepare tracker before process");
    break;
  }
  case Status::INITIALIZING: {
    if (NO_IMU) {
      auto  Tc0b = currFrame->getTbc(0).inverse();
      auto& Twb  = currFrame->getTwb();
      Twb        = Tc0b;

      Eigen::Vector3d X = Eigen::Vector3d::UnitY();
      X *= -3.14 / 180 * 20;
      Twb.so3() *= Sophus::SO3d::exp(X);
    }

    mLocalMap->addFrame(currFrame);

    currFrame->setKeyFrame();
    int  createMPCount = initializeMapPoints(currFrame);
    bool OK            = createMPCount > Config::Vio::initializeMapPointCount;

    if (OK) {
      mStatus                            = Status::TRACKING;
      mNumCreatedPoints[currFrame->id()] = createMPCount;
    }
    else {
      mLocalMap->reset();
      return;
    }
    break;
  }
  case Status::TRACKING: {
    //YSTODO: changed if imu exists;
    if (NO_IMU) {
      auto& Twb = mLocalMap->getFrames().rbegin()->second->getTwb();
      currFrame->setTwb(Twb);
    }

    size_t connected = mLocalMap->addFrame(currFrame);

    //YSTODO: check quality with createdMPCount
    std::vector<db::Frame::Ptr>    frames;
    std::vector<db::MapPoint::Ptr> trackingMapPoints;
    mLocalMap->getCurrentStates(frames, trackingMapPoints);

    BasicSolver::solveFramePose(currFrame);

    mVioSolver->solve(frames, trackingMapPoints);
    auto& currFactorMap = currFrame->mapPointFactorMap(0u);
    float ratio         = float(connected) / float(currFactorMap.size());

    if (ratio < 0.8) {
      if (Config::Vio::debug)
        ToyLogD("{}th frame, mp ratio : {}= {} /{}",
                currFrame->id(),
                ratio,
                connected,
                currFrame->mapPointFactorMap(0).size());
      mSetKeyFrame = true;
    }

    if (mSetKeyFrame && mKeyFrameAfter > Config::Vio::newKeyFrameAfter) {
      int createMPCount = initializeMapPoints(currFrame);

      if (createMPCount > 0) {
        mNumCreatedPoints[currFrame->id()] = createMPCount;
        currFrame->setKeyFrame();
      }
    }

    if (currFrame->isKeyFrame()) {
      mKeyFrameAfter = 0;
    }
    else {
      ++mKeyFrameAfter;
    }

    //drawDebugView(100, 0);
    //DEBUG_POINT();
    //cv::waitKey(1);
    //
    //if (newMp > 0) && createMPCount > 0) {
    //    ToyLogD("     Set KeyFrame : {} create Mp Count : {}",
    //            currFrame->id(),
    //            createMPCount);
    //  }

    //if (currFrame->id() > 50) {
    //  drawDebugView(100, 0);
    //  DEBUG_POINT();
    //  cv::waitKey();
    //}

    //drawDebugView(101, 1040);
    //drawDebugView(101, 1040);

    if (frames.size() < Config::Vio::solverMinimumFrames) {
      return setDataToInfo();
    }

    selectMarginalFrame(frames);

    std::forward_list<db::MapPoint::Ptr> lostMapPoints;
    //for (auto& mp : trackingMapPoints) {
    //  if (!currFactorMap.count(mp->id())) {
    //    lostMapPoints.push_front(mp);
    //    mp->setState(db::MapPoint::Status::MARGINED);
    //  }
    //}

    for (auto id : mMarginalFrameIds) {
      mLocalMap->removeFrame(id);
    }

    mVioSolver->marginalize(mMarginalKeyFrameIds, lostMapPoints);

    for (auto id : mMarginalKeyFrameIds) {
      mNumCreatedPoints.erase(id);
      mLocalMap->removeFrame(id);
    }

    mMarginalFrameIds.clear();
    mMarginalKeyFrameIds.clear();
    break;
  }
  }

  setDataToInfo();
}

int LocalTracker::initializeMapPoints(std::shared_ptr<db::Frame> currFrame) {
  auto& mpCands = mLocalMap->getMapPointCandidiates();

  int initCount = 0;
  int oldCount  = 0;
  int tryCount  = mpCands.size();

  FrameCamId        frameCamId0{currFrame->id(), 0};
  std::set<int64_t> eraseMpIds;

  //YSTODO : tbb
  for (auto& [mpId, mp] : mpCands) {
    auto& frameFactorMap = mp->frameFactorMap();

    if (frameFactorMap.count(frameCamId0) == 0) {
      ++oldCount;
      eraseMpIds.insert(mpId);
      continue;
    }

    auto& factor0 = frameFactorMap[frameCamId0];

    bool initSuccess = false;
    for (auto& [frameCamId1, factor1] : frameFactorMap) {
      if (frameCamId0 == frameCamId1) {
        continue;
      }

      Eigen::Vector3d Pc0x;
      switch (factor1.type()) {
      case db::Factor::Type::REPROJECTION: {
        auto Twc0  = factor0.frame()->getTwc(frameCamId0.camId);
        auto Twc1  = factor1.frame()->getTwc(frameCamId1.camId);
        auto Tc1c0 = Twc1.inverse() * Twc0;

        //auto image0 = factor0.frame()
        //                ->getImagePyramid(frameCamId0.camId)
        //                ->getOrigin()
        //                .clone();
        //auto image1 = factor1.frame()
        //                ->getImagePyramid(frameCamId1.camId)
        //                ->getOrigin()
        //                .clone();

        //cv::cvtColor(image0, image0, CV_GRAY2BGR);
        //cv::cvtColor(image1, image1, CV_GRAY2BGR);
        //cv::circle(image0,
        //           cv::Point2f(factor0.uv().x(), factor0.uv().y()),
        //           4,
        //           {255, 0, 0},
        //           -1);
        //cv::circle(image1,
        //           cv::Point2f(factor1.uv().x(), factor1.uv().y()),
        //           4,
        //           {255, 0, 0},
        //           -1);
        //cv::imshow("0", image0);
        //cv::imshow("1", image1);
        //cv::waitKey();

        if (Tc1c0.translation().squaredNorm() < 0.0025)
          continue;

        initSuccess |= BasicSolver::triangulate(factor0.undist(),
                                                factor1.undist(),
                                                Tc1c0,
                                                Pc0x);

        break;
      }
      case db::Factor::Type::DEPTH: {
        TOY_ASSERT_MESSAGE(false, "not implemented");
        break;
      }
      default: {
        TOY_ASSERT_MESSAGE(false, "this should not happen");
        break;
      }
      }

      if (initSuccess) {
        mp->setHost(currFrame);
        double          invD = 1.0 / Pc0x.z();
        Eigen::Vector2d nuv  = Pc0x.head(2) * invD;
        mp->setUndist(nuv);
        mp->setInvDepth(invD);
        mp->setState(db::MapPoint::Status::TRACKING);
        mLocalMap->addMapPoint(mp);

        ++initCount;
        break;
      }
    }

    if (initSuccess) {
      eraseMpIds.insert(mpId);
    }
  }

  for (auto& id : eraseMpIds) {
    mpCands.erase(id);
  }
  auto candSize = mpCands.size();

  if (Config::Vio::debug) {
    ToyLogD("init : {}, oldCount :{}, cand : {} -> {}",
            initCount,
            oldCount,
            tryCount,
            candSize);
  }

  return initCount;
}

void LocalTracker::selectMarginalFrame(std::vector<db::Frame::Ptr>& allFrames) {
  std::vector<db::Frame::Ptr> keyFrames;
  keyFrames.reserve(Config::Vio::maxKeyFrameSize);

  auto latestFrame = allFrames.back();

  for (auto& frame : allFrames) {
    if (frame->isKeyFrame()) {
      keyFrames.push_back(frame);
    }
    else {
      if (frame->id() != latestFrame->id()) {
        mMarginalFrameIds.push_back(frame->id());
      }
    }
  }

  auto lastKeyFrame = keyFrames.back();
  auto endIt        = std::prev(keyFrames.end(), 2);

  while (keyFrames.size() - mMarginalKeyFrameIds.size() > Config::Vio::maxKeyFrameSize) {
    std::map<int64_t, int> connectedMapPoints;
    for (auto& map : latestFrame->mapPointFactorMaps()) {
      for (auto& [mpId, f] : map) {
        if (f.mapPoint()->status() < db::MapPoint::Status::TRACKING)
          continue;
        connectedMapPoints[f.mapPoint()->hostFrame()->id()]++;
      }
    }

    bool selected = false;

    for (auto it = keyFrames.begin(); it < endIt; ++it) {
      auto kfId = (*it)->id();

      int count = 0;
      if (connectedMapPoints.count(kfId)) {
        count = connectedMapPoints[kfId];
      }

      float ratio = float(count) / mNumCreatedPoints[kfId];
      ToyLogD("marg due to ratio : {} / {} = {} id: {}",
              count,
              mNumCreatedPoints[kfId],
              ratio,
              kfId);
      if (ratio < Config::Vio::margFeatureConnectionRatio) {
        /*ToyLogD("marg due to ratio : {} / {} = {} id: {}",
                count,
                mNumCreatedPoints[kf->id()],
                ratio,
                kf->id());*/
        mMarginalKeyFrameIds.emplace(kfId);
        selected = true;
        ToyLogD("marginalize due to : ratio  id : {}", kfId);
        break;
      }
    }

    if (selected) {
      continue;
    }

    float          minScore = std::numeric_limits<float>::max();
    db::Frame::Ptr minKeyFrame;
    int64_t        minId = -1;

    for (auto it1 = keyFrames.begin(); it1 < endIt; ++it1) {
      float denom = 0;
      for (auto it2 = keyFrames.begin(); it2 < endIt; ++it2) {
        // clang-format off
        denom += 1.0f
                 / (
                   ((*it1)->getTwb().translation() - (*it2)->getTwb().translation()).norm()
                    + 0.00001f
                   );
        // clang-format on
      }
      // clang-format off
      float score = std::sqrt(
        ((*it1)->getTwb().translation() - lastKeyFrame->getTwb().translation()) .norm()
      ) * denom;

      ToyLogD("id : {} marg distance score : {} denom : {}", (*it1)->id(), (int)score, denom);
      // clang-format on
      if (score < minScore) {
        minId    = (*it1)->id();
        minScore = score;
      }
    }

    TOY_ASSERT(minId >= 0);

    ToyLogD("marginalize due to : distance score id : {}", minId);
    mMarginalKeyFrameIds.emplace(minId);
  }
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
  auto& frames = mLocalMap->getFrames();

  if (frames.empty()) {
    return;
  }

  int xOffset = frames.begin()->second->getImagePyramid(0)->getOrigin().cols;
  int yOffset = frames.begin()->second->getImagePyramid(0)->getOrigin().rows;

  int k  = 0;
  int id = 0;
  for (auto& [fid, f] : frames) {
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