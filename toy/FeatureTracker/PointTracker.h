#pragma once
#include <string>
#include <vector>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace toy {
class Camera;
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
  size_t extract(db::Frame* frame);

  void devideImage(cv::Mat&                  src,
                   cv::Mat&                  mask,
                   std::vector<cv::Mat>&     subs,
                   std::vector<cv::Point2i>& offsets);

  void convertCVKeyPointsToFeature(Camera*                    cam,
                                   std::vector<cv::KeyPoint>& kpts,
                                   db::Feature*               feature);

  size_t track(db::Frame* prev, db::Frame* curr);
  size_t trackStereo(db::Frame* frame);

  static cv::Mat createMask(const cv::Mat& origin, db::Feature* feature);

protected:
  size_t                 mFeatureId;
  size_t                 mMaxFeatureSize;
  std::string            mType;
  cv::Ptr<cv::Feature2D> mFeature2D;
  std::set<int>          mPrevIds;
};
}  //namespace toy