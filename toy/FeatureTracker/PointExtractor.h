#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace toy {
namespace db {
class Feature;
}
class PointExtractor {
public:
  PointExtractor(std::string type);
  ~PointExtractor();

  void process(cv::Mat& image, db::Feature* points);

protected:
  int                    mFeatureId = -1;
  std::string            mType      = "";
  cv::Ptr<cv::Feature2D> mFeature2D;
  void setKptsToFeature(std::vector<cv::KeyPoint>& kpts, db::Feature* feature);
};
}  //namespace toy