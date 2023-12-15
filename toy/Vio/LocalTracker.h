#pragma once
#include <array>
#include "Thread.h"

namespace toy {
class VioSolver;
class BasicSolver;
namespace db {
class LocalMap;
class Frame;
}  //namespace db

class FeatureTracker;
class FrameSolver;
class LocalTracker : public Thread<db::Frame, void> {
public:
  using Thread<db::Frame, void>::registerOutQueue;
  using Thread<db::Frame, void>::insert;
  LocalTracker();
  ~LocalTracker();
  void prepare();
  void process();

private:
  using Thread<db::Frame, void>::getLatestInput;
  using Thread<db::Frame, void>::mInQueue;

  int  initializeMapPoints(std::shared_ptr<db::Frame> currFrame);
  int  getMarginalFrameId(std::vector<std::shared_ptr<db::Frame>>& frames);
  void setDataToInfo();

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };
  Status                        mStatus;
  std::unique_ptr<db::LocalMap> mLocalMap;
  std::unique_ptr<VioSolver>    mVioSolver;
};

}  //namespace toy