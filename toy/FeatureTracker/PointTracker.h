#pragma once
namespace toy {
class PointExtractor;
class PointMatcher;
class PointTracker {
public:
  PointTracker() = default;
  ~PointTracker() = default;

protected:
  PointExtractor* extractor;
  PointMatcher*   matcher;
};
}  //namespace toy
