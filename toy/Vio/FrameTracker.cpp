#include <opencv2/opencv.hpp>

#include "config.h"
#include "ToyLogger.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "MemoryPointerPool.h"
#include "LocalMap.h"
#include "FeatureTracker.h"
#include "VioSolver.h"
#include "FrameTracker.h"

namespace toy {
FrameTracker::FrameTracker()
  : mFeatureTracker{nullptr}
  , mFrameSolver{nullptr}
  , mStatus{Status::NONE}
  , mPrevFrame{nullptr} {}

FrameTracker::~FrameTracker() {
  delete mFeatureTracker;
  mFeatureTracker = nullptr;

  delete mFrameSolver;
  mFrameSolver = nullptr;
}

void FrameTracker::prepare() {
  mFeatureTracker =
    new FeatureTracker(Config::Vio::pointTracker, Config::Vio::lineTracker);

  //mVioSolver = VioSolverFactory::createVioSolver();
  mStatus = Status::INITIALIZING;
}

void FrameTracker::process() {
  db::Frame* currFrame = getLatestFrame();
  if (!currFrame) return;

  bool OK{false};
  OK = mFeatureTracker->process(mPrevFrame, currFrame);

  if (!OK) { ToyLogD("Do something like reject frame .. ") };

  switch (mStatus) {
  case Status::NONE: {
    break;
  }
  case Status::INITIALIZING: {
    mStatus = Status::TRACKING;
    //landmark initialize...? but dont add to map!
    break;
  }
  case Status::TRACKING: {
    if (Config::Vio::frameTrackerSolvePose) { trackPose(); }
    db::MemoryPointerPool::release(mPrevFrame);
    break;
  }
  }

  db::Frame* out = currFrame->clone();
  mOutQueue->push(out);

  mPrevFrame = currFrame;
}

db::Frame* FrameTracker::getLatestFrame() {
  db::ImagePyramid* pyramids = getLatestInput();
  if (!pyramids) return nullptr;

  db::Frame* currFrame = db::MemoryPointerPool::createFramePtr(pyramids);

  Camera* cam0 = CameraFactory::createCamera(&Config::Vio::camInfo0);
  Camera* cam1 = CameraFactory::createCamera(&Config::Vio::camInfo1);

  currFrame->setCameras(cam0, cam1);
  currFrame->setLbc(Config::Vio::camInfo0.Mbc.data(), Config::Vio::camInfo1.Mbc.data());

  return currFrame;
}

void FrameTracker::trackPose() {
  ToyLogW("Not implemented yet");
}

}  //namespace toy