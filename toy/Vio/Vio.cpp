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
  , currMainImage{nullptr} {
  mLocalMap = new db::LocalMap();
}

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
}

void Vio::process() {
  db::ImagePyramid* pyramids = getLatestInput();
  db::Frame* currFrame = db::MemoryPointerPool::getInstance()->createFrame(pyramids);

  Camera* cam0 = CameraFactory::createCamera(&Config::Vio::camInfo0);
  Camera* cam1 = CameraFactory::createCamera(&Config::Vio::camInfo1);
  currFrame->setCameras(cam0, cam1);
  currFrame->setLbc(Config::Vio::camInfo0.Mbc.data(), Config::Vio::camInfo1.Mbc.data());

  mFeatureTracker->process(currFrame);
}

}  //namespace toy