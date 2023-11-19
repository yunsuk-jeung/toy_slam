#pragma once
#include <string>
namespace toy {
class Frame;
class PointTracker;
class LineTracker;

class FeatureTracker {
public:
  FeatureTracker() = delete;
  FeatureTracker(std::string pointTracker, std::string lineTracker);

  ~FeatureTracker();

  //reject frame when process is false?
  bool process(db::Frame* frame);

protected:
  PointTracker* mPointTracker;
  LineTracker*  mLineTracker;
};
}  //namespace toy