#pragma once
#include <memory>
#include <string>
namespace toy {
namespace db {
class Frame;
}
class PointTracker;
class LineTracker;

class FeatureTracker {
public:
  FeatureTracker() = delete;
  FeatureTracker(std::string pointTracker, std::string lineTracker);

  ~FeatureTracker();

  //reject frame when process is false?
  bool process(db::FrameState* prevFrameState, db::FrameState* currFrameState);

protected:
  std::unique_ptr<PointTracker> mPointTracker;
  std::unique_ptr<LineTracker>  mLineTracker;
};
}  //namespace toy