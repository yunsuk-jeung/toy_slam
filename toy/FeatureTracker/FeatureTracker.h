#pragma once
#include <string>
namespace toy {
class Frame;
class PointExtractor;
class PointMatcher;
class LineExtractor;
class LineMatcher;

class FeatureTracker {
public:
  FeatureTracker() = delete;
  FeatureTracker(std::string pointExtractor,
                 std::string pointMatcher,
                 std::string mLineExtractor = "none",
                 std::string mLineMatcher   = "none");

  ~FeatureTracker();

  //reject frame when process is false?
  bool process(Frame* frame);

protected:
  PointExtractor* mPointExtractor;
  PointMatcher*   mPointMatcher;
  LineExtractor*  mLineExtractor;
  LineMatcher*    mLineMatcher;
};
}  //namespace toy