#pragma once
#include <string>
#include <vector>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace toy {
class Camera;
class PointMatcher;
namespace db {
class Feature;
class Frame;
}  //namespace db
class PointTracker {
public:
  PointTracker(std::string type);
  ~PointTracker();

  size_t process(db::Frame* prev, db::Frame* curr);

protected:
  size_t detect(db::Frame* frame);

  void devideImage(cv::Mat&                  src,
                   std::vector<cv::Mat>&     subs,
                   std::vector<cv::Point2i>& offsets);

  void devideImage(cv::Mat&                  src,
                   cv::Mat&                  mask,
                   std::vector<cv::Mat>&     subs,
                   std::vector<cv::Point2i>& offsets);

  void convertCVKeyPointsToFeature(Camera*                    cam,
                                   std::vector<cv::KeyPoint>& kpts,
                                   db::Feature*               feature);

  size_t match(db::Frame* prev, db::Frame* curr);
  size_t matchStereo(db::Frame* frame);

  void           checkEmptyGrid(const cv::Mat& origin, db::Feature* feature);
  static cv::Mat createMask(const cv::Mat& origin, db::Feature* feature);

protected:
  size_t                        mFeatureId;
  size_t                        mMaxFeatureSize;
  std::vector<uint8_t>          mGridStatus;
  std::string                   mFeatureType;
  std::string                   mMatcherType;
  cv::Ptr<cv::Feature2D>        mPointDetector;
  std::shared_ptr<PointMatcher> mPointMatcher;
  //std::set<int>                 mPrevIds;
  int                           mStereoTrackingIntervalCount;
};
}  //namespace toy