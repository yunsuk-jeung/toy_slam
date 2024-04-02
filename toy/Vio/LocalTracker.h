#pragma once
#include <array>
#include <map>
#include "Thread.h"

namespace toy {
class VioSolver;
class BasicSolver;
namespace db {
class LocalMap;
class FrameState;
}  //namespace db

class FeatureTracker;
class FrameSolver;
class LocalTracker : public Thread<db::FrameState, void> {
public:
  using Thread<db::FrameState, void>::registerOutQueue;
  using Thread<db::FrameState, void>::insert;
  LocalTracker();
  ~LocalTracker();
  void prepare();
  void process();

private:
  using Thread<db::FrameState, void>::getLatestInput;
  using Thread<db::FrameState, void>::in_queue_;

  //int                        initializeMapPoints(std::shared_ptr<db::Frame> currFrame);
  int initializeMapPoints(std::shared_ptr<db::FrameState> currFrame);
  std::shared_ptr<db::FrameState> selectMarginalFrame(
    std::vector<std::shared_ptr<db::FrameState>>& frames);
  void setDataToInfo();

  void drawDebugView(int tag, int offset = 0);

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };
  Status                        mStatus;
  std::unique_ptr<db::LocalMap> mLocalMap;
  std::unique_ptr<VioSolver>    mVioSolver;
  int                           mKeyFrameAfter;
  std::map<int64_t, int>        mNumCreatedPoints;
};

}  //namespace toy