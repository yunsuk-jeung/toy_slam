#pragma once
#include <array>
#include <set>
#include <vector>
#include <map>
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
  using Thread<db::Frame, void>::in_queue_;

  int  initializeMapPoints(std::shared_ptr<db::Frame> currFrame);
  void selectMarginalFrame(std::vector<std::shared_ptr<db::Frame>>& frames);
  void setDataToInfo();

  void drawDebugView(int tag, int offset = 0);

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };
  Status                        mStatus;
  std::unique_ptr<db::LocalMap> mLocalMap;
  std::unique_ptr<VioSolver>    mVioSolver;
  int                           mKeyFrameAfter;
  std::map<int64_t, int>        mNumCreatedPoints;
  bool                          mSetKeyFrame;

  std::vector<int64_t> mMarginalFrameIds;
  std::set<int64_t>    mMarginalKeyFrameIds;
};

}  //namespace toy