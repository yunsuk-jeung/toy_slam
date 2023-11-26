#pragma once
#include <array>
#include "Thread.h"
#include "Frame.h"

namespace toy {
namespace db {
LocalMap;
}

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

  bool initialize(db::Frame*);

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };
  Status        mStatus;
  db::LocalMap* mLocalMap;
};

}  //namespace toy