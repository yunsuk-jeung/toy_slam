#include <opencv2/opencv.hpp>

#include "config.h"
#include "ToyLogger.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
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
  mFeatureTracker = new FeatureTracker(Config::Vio::pointTracker,
                                       Config::Vio::lineTracker);

  mStatus = Status::INITIALIZING;
}

void FrameTracker::process() {
  db::Frame::Ptr currFrame = getLatestFrame();
  if (!currFrame)
    return;

  bool OK{false};
  OK = mFeatureTracker->process(mPrevFrame.get(), currFrame.get());

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
    //db::MemoryPointerPool::release(mPrevFrame);
    break;
  }
  }

  db::Frame::Ptr out = currFrame->clonePtr();

  mOutQueue->push(out);
  mPrevFrame = currFrame;
}

db::Frame::Ptr FrameTracker::getLatestFrame() {
  db::ImagePyramidSet::Ptr set = getLatestInput();
  if (!set)
    return nullptr;

  Camera* cam0 = CameraFactory::createCamera(&Config::Vio::camInfo0);
  Camera* cam1 = CameraFactory::createCamera(&Config::Vio::camInfo1);

  db::Frame::Ptr currFrame = std::make_shared<db::Frame>(set);
  currFrame->setCameras(cam0, cam1);
  currFrame->setSbc(Config::Vio::camInfo0.Mbc.data(), Config::Vio::camInfo1.Mbc.data());

  return currFrame;
}

void FrameTracker::trackPose() {
  ToyLogW("Not implemented yet");
}

}  //namespace toy