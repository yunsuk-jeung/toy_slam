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
class PointExtractor {
public:
  PointExtractor(std::string type);
  ~PointExtractor();

  void process(db::Frame* frame);

protected:
  static cv::Mat createMask();

  void convertCVKeyPointsToFeature(Camera*                    cam,
                                   std::vector<cv::KeyPoint>& kpts,
                                   db::Feature*               feature);

protected:
  int                    mFeatureId = -1;
  std::string            mType      = "";
  cv::Ptr<cv::Feature2D> mFeature2D;
};
}  //namespace toy