#include "config.h"
#include "FeatureTracker.h"
#include "VioSolverFactory.h"
#include "VioSolver.h"
#include "Vio.h"
#include "LocalMap.h"

namespace toy {
Vio::Vio() : mFeatureTracker{nullptr}, mVioSolver{nullptr} {
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
  db::ImagePyramid* currImage = getLatestInput();
  if (!currImage) return;

  db::Frame* currFrame = mLocalMap->createNewFrame(currImage);
};

}  //namespace toy