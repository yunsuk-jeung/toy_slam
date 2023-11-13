#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace toy {
class PointExtractor {
public:
  PointExtractor();
  ~PointExtractor();

  void process(cv::Mat image, std::vector<cv::Point2f>& points);

protected:
  cv::Ptr<cv::Feature2D> extractor;
  static void            convertKptsToPts(std::vector<cv::KeyPoint>& kpts,
                                          std::vector<cv::Point2f>&  pts);
};
}  //namespace toy