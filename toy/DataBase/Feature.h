#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
namespace toy {
namespace db {
class Feature {
public:
  class Keypoints {
  public:
    size_t size() { return mIds.size(); }

    void reserve(size_t size) {
      mIds.reserve(size);
      mLevels.reserve(size);
      mPoints.reserve(size);
      mTrackCounts.reserve(size);
      mUndsits.reserve(size);
    }

    void push_back(Keypoints& kpts) {

      // clang-format off
      mIds.insert(mIds.end(), kpts.mIds.begin(), kpts.mIds.begin());
      mLevels.insert(mLevels.end(), kpts.mLevels.begin(), kpts.mLevels.begin());
      mPoints.insert(mPoints.end(), kpts.mPoints.begin(), kpts.mPoints.begin());
      mTrackCounts.insert(mTrackCounts.end(), kpts.mTrackCounts.begin(), kpts.mTrackCounts.begin());
      mUndsits.insert(mUndsits.end(), kpts.mUndsits.begin(), kpts.mUndsits.begin());
      // clang-format on
    }

    std::vector<uint32_t>    mIds;
    std::vector<uint32_t>    mLevels;
    std::vector<cv::Point2f> mPoints;
    std::vector<uint32_t>    mTrackCounts;
    std::vector<cv::Point2f> mUndsits;
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