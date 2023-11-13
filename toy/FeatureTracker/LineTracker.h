#pragma once
namespace toy {
class LineExtractor;
class LineMatcher;
class LineTracker {
public:
  LineTracker();
  ~LineTracker();

protected:
  LineExtractor* extractor;
  LineMatcher*   matcher;
};
}  //namespace toy
