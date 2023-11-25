#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
namespace toy {
namespace db {
class Feature {
public:
  class Keypoints {
  public:
    Keypoints()  = default;
    ~Keypoints() = default;
    Keypoints(const Keypoints& src) {
      mIds         = src.mIds;
      mLevels      = src.mLevels;
      mUvs         = src.mUvs;
      mTrackCounts = src.mTrackCounts;
      mUndists     = src.mUndists;
      mStatus      = src.mStatus;
    }
    size_t size() { return mIds.size(); }

    void reserve(size_t size) {
      mIds.reserve(size);
      mLevels.reserve(size);
      mUvs.reserve(size);
      mTrackCounts.reserve(size);
      mUndists.reserve(size);
      mStatus.reserve(size);
    }

    void push_back(Keypoints& kpts) {
      // clang-format off
      mIds.insert(mIds.end(), kpts.mIds.begin(), kpts.mIds.begin());
      mLevels.insert(mLevels.end(), kpts.mLevels.begin(), kpts.mLevels.begin());
      mUvs.insert(mUvs.end(), kpts.mUvs.begin(), kpts.mUvs.begin());
      mTrackCounts.insert(mTrackCounts.end(), kpts.mTrackCounts.begin(), kpts.mTrackCounts.begin());
      mUndists.insert(mUndists.end(), kpts.mUndists.begin(), kpts.mUndists.begin());
      mStatus.insert(mStatus.end(), kpts.mStatus.begin(), kpts.mStatus.begin());
      // clang-format on
    }

    std::vector<uint32_t>    mIds;
    std::vector<uint32_t>    mLevels;
    std::vector<cv::Point2f> mUvs;
    std::vector<uint32_t>    mTrackCounts;
    std::vector<cv::Point2f> mUndists;
    std::vector<uint8_t>     mStatus;
  };

  Feature()  = default;
  ~Feature() = default;

  Feature(const Feature* src) { this->mKeypoints = src->mKeypoints; }

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