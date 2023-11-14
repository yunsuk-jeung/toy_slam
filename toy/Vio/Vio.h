#pragma once

#include "Frame.h"
#include "Processor.h"

namespace toy {
namespace db {
class ImagePyramid;
class Frame;
}  //namespace db

class FeatureTracker;
class VioSolver;
class Vio : public Processor<db::ImagePyramid, void> {
public:
  Vio();
  ~Vio();

  void prepare();

private:
  using Processor<db::ImagePyramid, void>::mInQueue;

  std::unique_ptr<FeatureTracker> mFeatureTrackerUptr;
  std::unique_ptr<VioSolver>      mVioSolverUptr;
};

}  //namespace toy