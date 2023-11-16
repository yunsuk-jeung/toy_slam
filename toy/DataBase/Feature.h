#pragma once
#include <opencv2/core.hpp>

namespace toy {
namespace db {
class Feature {
public:
  class Keypoints {
  public:
    size_t size() { return mIds.size(); }
    void   reserve(size_t size) {
      mIds.reserve(size);
      mLevels.reserve(size);
      mPoints.reserve(size);
      mTrackCounts.reserve(size);
    }

    std::vector<uint32_t>    mIds;
    std::vector<uint32_t>    mLevels;
    std::vector<cv::Point2f> mPoints;
    std::vector<uint32_t>    mTrackCounts;
  };
  Feature()  = default;
  ~Feature() = default;

protected:
  Keypoints mKeypoints;

  //struct Lines {
  //  std::vector<uint32_t>    ids;
  //  std::vector<uint32_t>    levels;
  //  std::vector<cv::Point2f> start;
  //  std::vector<cv::Point2f> end;

  //} lines;

public:
  Keypoints& getKeypoints() { return mKeypoints; }
};
}  //namespace db
}  //namespace toy