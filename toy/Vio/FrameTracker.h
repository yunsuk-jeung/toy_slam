#pragma once
#include <memory>
#include <array>
#include "ImagePyramid.h"
#include "Thread.h"

namespace toy {
namespace db {
class Frame;
}
class FeatureTracker;
class FrameTracker : public Thread<db::ImagePyramidSet, db::Frame> {
public:
  using Thread<db::ImagePyramidSet, db::Frame>::registerOutQueue;
  using Thread<db::ImagePyramidSet, db::Frame>::insert;

  FrameTracker();
  ~FrameTracker();
  void prepare();
  void process();

private:
  using Thread<db::ImagePyramidSet, db::Frame>::getLatestInput;
  using Thread<db::ImagePyramidSet, db::Frame>::in_queue_;

  std::shared_ptr<db::Frame> getLatestFrame();
  void                       trackPose();

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };

  Status                     mStatus;
  FeatureTracker*            mFeatureTracker;
  std::shared_ptr<db::Frame> mPrevFrame;
};

}  //namespace toy