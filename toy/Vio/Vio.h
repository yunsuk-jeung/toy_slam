#pragma once

#include "Frame.h"
#include "Processor.h"

namespace toy {
class FeatureTracker;
class VioSolver;
class Vio {
public:
  Vio();
  ~Vio();

  void prepare();

private:
  std::unique_ptr<FeatureTracker> featureTrackerUptr;
  std::unique_ptr<VioSolver>      vioSolverUptr;
};

}  //namespace toy