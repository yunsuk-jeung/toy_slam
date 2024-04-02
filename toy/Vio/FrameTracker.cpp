#include <opencv2/opencv.hpp>

#include "config.h"
#include "ToyLogger.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "FrameState.h"
#include "LocalMap.h"
#include "FeatureTracker.h"
#include "VioSolver.h"
#include "FrameTracker.h"

namespace toy {
FrameTracker::FrameTracker()
  : mFeatureTracker{nullptr}
  , mFrameSolver{nullptr}
  , mStatus{Status::NONE}
  , mPrevFrameState{nullptr} {}

FrameTracker::~FrameTracker() {
  delete mFeatureTracker;
  mFeatureTracker = nullptr;

  delete mFrameSolver;
  mFrameSolver = nullptr;
}

void FrameTracker::prepare() {
  mFeatureTracker = new FeatureTracker(Config::Vio::pointTracker,
                                       Config::Vio::lineTracker);

  mStatus = Status::INITIALIZING;
}

void FrameTracker::process() {
  auto currFrameState = getLatestFrameState();
  if (!currFrameState)
    return;

  bool OK{false};
  OK = mFeatureTracker->process(mPrevFrameState.get(), currFrameState.get());

  if (!OK) {
    ToyLogD("Do something like reject frame .. ")
  };

  switch (mStatus) {
  case Status::NONE: {
    break;
  }
  case Status::INITIALIZING: {
    mStatus = Status::TRACKING;
    break;
  }
  case Status::TRACKING: {
    if (Config::Vio::frameTrackerSolvePose) {
      trackPose();
    }
    //db::MemoryPointerPool::release(mPrevFrameState);
    break;
  }
  }

  auto out = currFrameState->clone();

  out_queue_->push(out);
  mPrevFrameState = currFrameState;
}

db::FrameState::Ptr FrameTracker::getLatestFrameState() {
  db::ImagePyramidSet::Ptr set = getLatestInput();
  if (!set)
    return nullptr;

  std::vector<Camera::Ptr> cams;
  auto                     cam0 = CameraFactory::createCamera(&Config::Vio::camInfo0);
  auto                     cam1 = CameraFactory::createCamera(&Config::Vio::camInfo1);

  db::Frame::Ptr currFrame = std::make_shared<db::Frame>(set);
  currFrame->setCameras(cam0, cam1);
  currFrame->setSbc(Config::Vio::camInfo0.Mbc.data(), Config::Vio::camInfo1.Mbc.data());

  return currFrame;
}

void FrameTracker::trackPose() {
  ToyLogW("Not implemented yet");
}

}  //namespace toy