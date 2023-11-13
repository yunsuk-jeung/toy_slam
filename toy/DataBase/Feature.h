#pragma once
#include <opencv2/core.hpp>

namespace toy {
namespace db {
class Feature {
public:
  Feature()  = default;
  ~Feature() = default;

protected:
  struct Points {
    std::vector<uint32_t>    ids;
    std::vector<uint32_t>    levels;
    std::vector<cv::Point2f> points;
    std::vector<uint32_t>    trackCounts;
  } points;

  // struct Lines {
  //   std::vector<uint32_t>    ids;
  //   std::vector<uint32_t>    levels;
  //   std::vector<cv::Point2f> start;
  //   std::vector<cv::Point2f> end;

  // } lines;
};
}  //namespace db
}  //namespace toy