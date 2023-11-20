#pragma once
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
  bool process(db::Frame* prevFrame, db::Frame* currentFrame);

protected:
  PointTracker* mPointTracker;
  LineTracker*  mLineTracker;
};
}  //namespace toy