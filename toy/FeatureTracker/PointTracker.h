#pragma once
#include <string>
#include <vector>
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

  void process(db::Frame* frame);

protected:
  size_t extract(db::Frame* frame);

  void devideImage(cv::Mat&                  src,
                   cv::Mat&                  mask,
                   std::vector<cv::Mat>&     subs,
                   std::vector<cv::Point2i>& offsets);

  void convertCVKeyPointsToFeature(Camera*                    cam,
                                   std::vector<cv::KeyPoint>& kpts,
                                   db::Feature*               feature);

  size_t track(db::Frame* frame);
  size_t trackStereo(db::Frame* frame);

  static cv::Mat createMask();

protected:
  int                    mFeatureId = -1;
  std::string            mType      = "";
  cv::Ptr<cv::Feature2D> mFeature2D;

  db::Frame* prevFrame;
};
}  //namespace toy