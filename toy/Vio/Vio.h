#pragma once

#include "Processor.h"

namespace toy {
namespace db {
class ImagePyramid;
class Frame;
class LocalMap;
}  //namespace db

class FeatureTracker;
class VioSolver;
class Vio : public Processor<db::ImagePyramid, void> {
public:
  using Processor<db::ImagePyramid, void>::registerOutQueue;
  using Processor<db::ImagePyramid, void>::insert;

  Vio();
  ~Vio();
  void prepare();
  void process();

private:
  using Processor<db::ImagePyramid, void>::getLatestInput;
  using Processor<db::ImagePyramid, void>::mInQueue;

private:
  std::unique_ptr<FeatureTracker> mFeatureTrackerUptr;
  std::unique_ptr<VioSolver>      mVioSolverUptr;
  std::unique_ptr<db::LocalMap>   mLocalMapUptr;
};

}  //namespace toy