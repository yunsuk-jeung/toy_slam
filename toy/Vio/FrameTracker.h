#pragma once
#include <memory>
#include <array>
#include "ImagePyramid.h"
#include "Thread.h"

namespace toy {
namespace db {
class FrameState;
}
class FeatureTracker;
class FrameSolver;
class FrameTracker : public Thread<db::ImagePyramidSet, db::FrameState> {
public:
  using Thread<db::ImagePyramidSet, db::FrameState>::registerOutQueue;
  using Thread<db::ImagePyramidSet, db::FrameState>::insert;

  FrameTracker();
  ~FrameTracker();
  void prepare();
  void process();

private:
  using Thread<db::ImagePyramidSet, db::FrameState>::getLatestInput;
  using Thread<db::ImagePyramidSet, db::FrameState>::in_queue_;

  std::shared_ptr<db::FrameState> getLatestFrameState();
  void                            trackPose();

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };

  Status                          mStatus;
  FeatureTracker*                 mFeatureTracker;
  FrameSolver*                    mFrameSolver;
  std::shared_ptr<db::FrameState> mPrevFrameState;
};

}  //namespace toy