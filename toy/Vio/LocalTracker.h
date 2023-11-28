#pragma once
#include <array>
#include "Thread.h"

namespace toy {
namespace db {
class LocalMap;
class Frame;
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

  bool initialize(std::shared_ptr<db::Frame>);

private:
  enum class Status { NONE = -1, INITIALIZING = 0, TRACKING = 1 };
  Status        mStatus;
  db::LocalMap* mLocalMap;
};

}  //namespace toy