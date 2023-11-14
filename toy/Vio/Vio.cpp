#include "Vio.h"
#include "config.h"
#include "FeatureTracker.h"
#include "VioSolver.h"
namespace toy {
Vio::Vio() : mFeatureTrackerUptr{nullptr}, mVioSolverUptr{nullptr} {}

Vio::~Vio() {}

void Vio::prepare() {
  auto solverType = static_cast<VioSolverFactory::SolverType>(Config::vioSolverType);
  mVioSolverUptr  = VioSolverFactory::createVioSolver(Config::useDouble, solverType);
};

}  //namespace toy