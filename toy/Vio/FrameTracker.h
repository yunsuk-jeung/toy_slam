1 #pragma once
#include <array>
#include "ImagePyramid.h"
#include "Thread.h"

  namespace toy {
  namespace db {
  class Frame;
  }
  class FeatureTracker;
  class FrameSolver;
  class FrameTracker : public Thread<db::ImagePyramid, db::Frame> {
  public:
    using Thread<db::ImagePyramid, db::Frame>::registerOutQueue;
    using Thread<db::ImagePyramid, db::Frame>::insert;

    FrameTracker();
    ~FrameTracker();
    void prepare();
    void process();

  private:
    using Thread<db::ImagePyramid, db::Frame>::getLatestInput;
    using Thread<db::ImagePyramid, db::Frame>::mInQueue;

    db::Frame* getLatestFrame();
    void       trackPose();

  private:
    enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };

    Status          mStatus;
    FeatureTracker* mFeatureTracker;
    FrameSolver*    mFrameSolver;
    db::Frame*      mPrevFrame;
  };

}  //namespace toy