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
  , mKeyFrameAfter{100} {}

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
      mStatus = Status::TRACKING;
    }
    else {
      mLocalMap->reset();
      return;
    }
    break;
  }
  case Status::TRACKING: {
    ++mKeyFrameAfter;

    //YSTODO: changed if imu exists;
    if (NO_IMU) {
      auto& Twb = mLocalMap->getFrames().rbegin()->second->getTwb();
      currFrame->setTwb(Twb);
    }

    size_t connected = mLocalMap->addFrame(currFrame);
    //if (mLocalMap->addFrame(currFrame)) {
    //mKeyFrameInterval = 0;
    //}

    //YSTODO: check quality with createdMPCount
    std::vector<db::Frame::Ptr>    frames;
    std::vector<db::MapPoint::Ptr> trackingMapPoints;
    std::vector<db::MapPoint::Ptr> marginedMapPoints;
    mLocalMap->getCurrentStates(frames, trackingMapPoints, marginedMapPoints);

    BasicSolver::solveFramePose(currFrame);

    //frames.front()->setFixed(true);
    mVioSolver->solve(frames, trackingMapPoints, {});

    bool setKf = false;

    //float ratio = float(connected) / float(mLocalMap->getMapPoints().size());
    float ratio = float(connected) / float(currFrame->getMapPointFactorMap().size());

    if (ratio < 0.8) {
      /*ToyLogD("{}th frame, mp ratio : {}= {} /{}",
              currFrame->id(),
              ratio,
              connected,
              currFrame->getMapPointFactorMap().size());*/
      setKf = true;
    }

    if (setKf) {
      int createMPCount = initializeMapPoints(currFrame);
      //ToyLogD("{}th frame, createMP : {} ", currFrame->id(), createMPCount);
      if (createMPCount > 10 && mKeyFrameAfter > 1) {
        currFrame->setKeyFrame();
        mKeyFrameAfter = 0;
      }
    }

    //if (newMp > 0) && createMPCount > 0) {
    //    ToyLogD("     Set KeyFrame : {} create Mp Count : {}",
    //            currFrame->id(),
    //            createMPCount);
    //  }

    if (currFrame->id() > 2800) {
      drawDebugView(100, 0);
      cv::waitKey();
    }

    //drawDebugView(101, 1040);
    //drawDebugView(101, 1040);

    if (frames.size() < Config::Vio::solverMinimumFrames)
      return setDataToInfo();

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

//int LocalTracker::initializeMapPoints(db::Frame::Ptr currFrame) {
//  auto& mapPointFactorMap = currFrame->getMapPointFactorMap();
//
//  auto            Twc0  = currFrame->getTwc(0);
//  auto            Twc1  = currFrame->getTwc(1);  //identity for mono or depth..
//  auto            Tc1c0 = Twc1.inverse() * Twc0;
//  Eigen::Vector3d Pc0x;
//
//  int successCount = 0;
//  int tryCount     = 0;
//  for (auto& [mpWeak, factor] : mapPointFactorMap) {
//    auto mp = mpWeak.lock();
//    if (mp->status() != db::MapPoint::Status::INITIALING)
//      continue;
//    ++tryCount;
//
//    switch (factor.type()) {
//    case db::ReprojectionFactor::Type::MONO: {
//      break;
//    }
//    case db::ReprojectionFactor::Type::STEREO: {
//      if (!BasicSolver::triangulate(factor.undist0(), factor.undist1(), Tc1c0, Pc0x)) {
//        continue;
//      }
//
//      double invD = 1.0 / Pc0x.z();
//      //Eigen::Vector2d nuv  = Pc0x.head(2) * invD;
//      //mp->setUndist(nuv);
//      mp->setInvDepth(invD);
//      mp->setState(db::MapPoint::Status::INITIALING);
//      ++successCount;
//      break;
//    }
//    case db::ReprojectionFactor::Type::DEPTH: {
//      break;
//    }
//    }
//  }
//
//  if (Config::Vio::debug) {
//    ToyLogD("triangulation successed {} / {}", successCount, tryCount);
//  }
//
//  if (Config::Vio::showStereoTracking) {
//    currFrame->drawReprojectionView(0, "tri0");
//    currFrame->drawReprojectionView(1, "tri1");
//    cv::waitKey();
//  }
//
//  return successCount;
//}

int LocalTracker::initializeMapPoints(std::shared_ptr<db::Frame> currFrame) {
  auto& mpCands = mLocalMap->getMapPointCandidiates();

  //cv::Mat image = currFrame->getImagePyramid(0)->getOrigin().clone();
  //cv::cvtColor(image, image, CV_GRAY2BGR);

  int initCount     = 0;
  int monoInitCount = 0;
  int tryCount      = 0;
  int candSize      = mpCands.size();
  for (auto it = mpCands.begin(); it != mpCands.end();) {
    bool  success      = false;
    auto& mp           = it->second;
    auto& frameFactors = mp->getFrameFactors();
    auto& factor       = frameFactors.back().second;
    auto& frame        = frameFactors.back().first.lock();

    Eigen::Vector3d Pc0x;
    switch (factor.type()) {
    case db::ReprojectionFactor::Type::STEREO: {
      auto Twc0  = frame->getTwc(0);
      auto Twc1  = frame->getTwc(1);  //identity for mono or depth..
      auto Tc1c0 = Twc1.inverse() * Twc0;

      if (!BasicSolver::triangulate(factor.undist0(), factor.undist1(), Tc1c0, Pc0x)) {
        //ToyLogE("stereo triangulation fail");
        ++it;
        continue;
      }
      ++initCount;
      success = true;
      break;
    }
    case db::ReprojectionFactor::Type::DEPTH: {
      TOY_ASSERT_MESSAGE(0, "not implemented");
      break;
    }
    default:
      auto frame0 = frameFactors.front().first.lock();
      if (frame == frame0) {
        ++it;
        continue;
      }
      auto  Twc0    = frame0->getTwc(0);
      auto& factor0 = frameFactors.front().second;

      auto Twc1  = frame->getTwc(0);
      auto Tc1c0 = Twc1.inverse() * Twc0;

      if (Tc1c0.translation().squaredNorm() < Config::Vio::minTriangulationBaselineSq) {
        ++it;
        continue;
      }

      if (!BasicSolver::triangulate(factor0.undist0(), factor.undist0(), Tc1c0, Pc0x)) {
        auto id = frameFactors.front().first.lock()->id();
        //ToyLogE("triangulation fail  frame {} -- frame {}", id, frame->id());
        ++it;
        continue;
      }
      //if (currFrame == frame) {
      //  auto cam = frame->getCamera(0);
      //  auto uv  = cam->project(Tc1c0 * Pc0x);
      //  auto uv2 = cam->project(factor.undist0());

      //cv::circle(image, uv, 5, {255, 0, 0}, -1);
      //cv::circle(image, uv, 3, {0, 0, 255}, -1);
      //}
      ++monoInitCount;

      success = true;
      break;
    }

    if (success) {
      double          invD = 1.0 / Pc0x.z();
      Eigen::Vector2d nuv  = Pc0x.head(2) * invD;

      mp->setUndist(nuv);
      mp->setInvDepth(invD);
      mp->setState(db::MapPoint::Status::INITIALING);
      mLocalMap->addMapPoint(mp);

      it = mpCands.erase(it);
    }
    else {
      ++it;
    }
  }
  //cv::imshow("test", image);
  //cv::waitKey();
  if (Config::Vio::debug) {
    ToyLogD("init stereo : {}, init mono : {},  cand :{}",
            initCount,
            monoInitCount,
            candSize);
  }
  return initCount;
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

  //if (frames.size() > Config::Vio::maxFrameSize) {
  //  return frames.front();
  //}

  if (keyFrames.size() > Config::Vio::maxKeyFrameSize) {
    auto& latestMap = keyFrames.back()->getMapPointFactorMap();

    for (auto& kf : keyFrames) {
      auto&  kfMap = kf->getMapPointFactorMap();
      size_t count = 0;

      for (auto& [mpWeak, f] : latestMap) {
        if (kfMap.count(mpWeak)) {
          ++count;
        }
      }

      float ratio = float(count) / kfMap.size();

      if (ratio < Config::Vio::margFeatureConnectionRatio) {
        //if (Config::Vio::debug)
        //std::stringstream ss;
        //for (auto& kff : keyFrames) {
        //  ss << std::setw(4) << kff->id();
        //}

        ToyLogD("marg due to ratio : {} / {} = {} id: {}",
                count,
                kfMap.size(),
                ratio,
                kf->id());
        return kf;
      }
    }

    //auto& oldest = keyFrames.front();
    //auto& target = *(std::prev(keyFrames.end(), 2));

    //auto& oldestMpFactorMap = oldest->getMapPointFactorMap();
    //auto& targetMpFactorMap = target->getMapPointFactorMap();

    //int count = 0;

    //for (auto& [mpWeak, f] : oldestMpFactorMap) {
    //  if (targetMpFactorMap.count(mpWeak)) {
    //    ++count;
    //  }
    //}

    //float ratio = float(count) / targetMpFactorMap.size();

    //if (ratio < Config::Vio::margFeatureConnectionRatio) {
    //  //if (Config::Vio::debug)
    //  ToyLogD("marginalize due to ratio : {} / {} = {}",
    //          count,
    //          targetMpFactorMap.size(),
    //          ratio);

    //return oldest;
    //}

    auto lastKeyFrame = keyFrames.back();

    float  minScore = std::numeric_limits<float>::max();
    size_t minIdx   = 1000000u;
    size_t idx      = 0u;

    auto endIt = std::prev(keyFrames.end(), 2);
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
      // clang-format on
      if (score < minScore) {
        minIdx   = idx;
        minScore = score;
      }
      idx++;
    }
    TOY_ASSERT(minIdx < 100);

    //if (Config::Vio::debug)
    ToyLogD("marginalize due to : distance score");

    return keyFrames[minIdx];
  }

  if (frames.size() > 1) {
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
    if (mpPtr->status() < db::MapPoint::Status::INITIALING)
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