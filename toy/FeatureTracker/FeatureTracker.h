#pragma once

namespace toy {
class Frame;
class PointTracker;
class LineTracker;
class FeatureTracker {
public:
  FeatureTracker();
  ~FeatureTracker();

  //reject frame when process is false?
  bool process(Frame* frame);

protected:
  PointTracker* pointTracker;
  LineTracker*  lineTracker;
};
}  //namespace toy