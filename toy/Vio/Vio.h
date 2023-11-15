#pragma once

#include "ImagePyramid.h"
#include "Processor.h"

namespace toy {
namespace db {
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

  Frame* getLatestFrame();

private:
  FeatureTracker* mFeatureTracker;
  VioSolver*      mVioSolver;
  db::LocalMap*   mLocalMap;

  db::ImagePyramid* currMainImage;
};

}  //namespace toy