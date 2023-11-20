#include <opencv2/opencv.hpp>

#include "config.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "MemoryPointerPool.h"
#include "LocalMap.h"
#include "FeatureTracker.h"
#include "VioSolver.h"
#include "Vio.h"

namespace toy {
Vio::Vio()
  : mFeatureTracker{nullptr}
  , mVioSolver{nullptr}
  , mLocalMap{new db::LocalMap()}
  , mStatus{Status::NONE} {}

Vio::~Vio() {
  delete mFeatureTracker;
  mFeatureTracker = nullptr;

  delete mVioSolver;
  mVioSolver = nullptr;

  delete mLocalMap;
  mLocalMap = nullptr;
}

void Vio::prepare() {
  mFeatureTracker =
    new FeatureTracker(Config::Vio::pointTracker, Config::Vio::lineTracker);

  mVioSolver = VioSolverFactory::createVioSolver();
  mStatus    = Status::INITIALIZING;
}

void Vio::process() {
  db::ImagePyramid* pyramids = getLatestInput();
  if (!pyramids) return;

  db::Frame* currFrame = db::MemoryPointerPool::getInstance()->createFrame(pyramids);
  Camera*    cam0      = CameraFactory::createCamera(&Config::Vio::camInfo0);
  Camera*    cam1      = CameraFactory::createCamera(&Config::Vio::camInfo1);
  currFrame->setCameras(cam0, cam1);
  currFrame->setLbc(Config::Vio::camInfo0.Mbc.data(), Config::Vio::camInfo1.Mbc.data());

  bool OK{false};
  switch (mStatus) {
  case Status::INITIALIZING:
    OK = mFeatureTracker->process(nullptr, currFrame);
    if (OK) mStatus = Status::TRACKING;
    break;
  case Status::TRACKING:
    OK = mFeatureTracker->process(mLocalMap->getLatestFrame(), currFrame);
    break;
  default:
    break;
  }

  mLocalMap->addFrame(currFrame);
}

}  //namespace toy