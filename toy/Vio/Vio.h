#pragma once
#include <array>
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

  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };

private:
  FeatureTracker* mFeatureTracker;
  VioSolver*      mVioSolver;
  db::LocalMap*   mLocalMap;
  Status          mStatus;
};

}  //namespace toy