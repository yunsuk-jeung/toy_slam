#include "config.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "LocalMap.h"
#include "FeatureTracker.h"
#include "VioSolverFactory.h"
#include "VioSolver.h"
#include "Vio.h"

namespace toy {
Vio::Vio() : mFeatureTracker{nullptr}, mVioSolver{nullptr}, currMainImage{nullptr} {
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
  mFeatureTracker = new FeatureTracker(Config::Vio::pointExtractor,
                                       Config::Vio::pointMatcher,
                                       Config::Vio::lineExtractor,
                                       Config::Vio::lineMatcher);

  mVioSolver = VioSolverFactory::createVioSolver();
}

void Vio::process() {
  db::Frame* currFrame = getLatestFrame();

  if (!currImage) return;
}

Frame* Vio::getLatestFrame() {
  //db::ImagePyramid* currImage = getLatestInput();
  //mLocalMap->createNewFrame(currImage);

  return nullptr;
};

}  //namespace toy