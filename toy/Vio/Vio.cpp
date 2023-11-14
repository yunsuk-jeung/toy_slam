#include "config.h"
#include "FeatureTracker.h"
#include "VioSolverFactory.h"
#include "VioSolver.h"
#include "Vio.h"
#include "LocalMap.h"

namespace toy {
Vio::Vio() : mFeatureTrackerUptr{nullptr}, mVioSolverUptr{nullptr} {
  mLocalMapUptr = std::make_unique<db::LocalMap>();
}

Vio::~Vio() {}

void Vio::prepare() {
  mFeatureTrackerUptr = std::make_unique<FeatureTracker>(Config::Vio::pointExtractor,
                                                         Config::Vio::pointMatcher,
                                                         Config::Vio::lineExtractor,
                                                         Config::Vio::lineMatcher);

  mVioSolverUptr = VioSolverFactory::createVioSolver();
}

void Vio::process() {
  db::ImagePyramid* currImage = getLatestInput();
  if (!currImage) return;

  db::Frame* currFrame = mLocalMapUptr->createNewFrame(currImage);
};

}  //namespace toy