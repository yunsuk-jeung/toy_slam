#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include "types.h"

namespace toy {
namespace db {
class ImagePyramid {
  
public:
  ImagePyramid(ImageType type, cv::Mat);
  ImagePyramid(ImageType type, uint8_t* buffer, int lenght, int width, int height);
  ~ImagePyramid();

protected:
  void        createImagePyrmid();
  static void convertToGray(cv::Mat& src, cv::Mat& dst);

protected:
  cv::Mat              origin;
  std::vector<cv::Mat> pyramids;

public:
  cv::Mat               getOrigin() { return origin; }
  std::vector<cv::Mat>& getPyramids() { return pyramids; }
};
};  //namespace db
}  //namespace toy