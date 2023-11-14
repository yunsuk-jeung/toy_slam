#pragma once
#include <memory>
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
  std::unique_ptr<PointExtractor> mPointExtractorUptr;
  std::unique_ptr<PointMatcher>   mPointMatcherUptr;
  std::unique_ptr<LineExtractor>  mLineExtractorUptr;
  std::unique_ptr<LineMatcher>    mLineMatcherUptr;
};
}  //namespace toy