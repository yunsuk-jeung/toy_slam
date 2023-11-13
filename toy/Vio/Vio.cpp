#include "Vio.h"
#include "config.h"
#include "FeatureTracker.h"
#include "VioSolver.h"
namespace toy {
Vio::Vio() : featureTrackerUptr{nullptr}, vioSolverUptr{nullptr} {}

Vio::~Vio() {}

void Vio::prepare() {
  auto solverType = static_cast<VioSolverFactory::SolverType>(Config::vioSolverType);
  vioSolverUptr   = VioSolverFactory::createVioSolver(Config::useDouble, solverType);
};

}  //namespace toy