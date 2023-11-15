#pragma once

#include "Processor.h"

namespace toy {
namespace db {
class Frame;
class LocalMap;
class ImagePyramid;
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
  FeatureTracker* mFeatureTracker;
  VioSolver*      mVioSolver;
  db::LocalMap*   mLocalMap;
};

}  //namespace toy