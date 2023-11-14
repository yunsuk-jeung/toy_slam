#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace toy {
class PointExtractor {
public:
  PointExtractor(std::string type);
  ~PointExtractor();

  void process(cv::Mat image, std::vector<cv::Point2f>& points);

protected:
  cv::Ptr<cv::Feature2D> mFeature2D;
  static void            convertKptsToPts(std::vector<cv::KeyPoint>& kpts,
                                          std::vector<cv::Point2f>&  pts);
};
}  //namespace toy