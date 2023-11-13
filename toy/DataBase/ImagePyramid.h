#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include "types.h"

namespace toy {
namespace db {
class ImagePyramid {

public:
  ImagePyramid(ImageType type, cv::Mat mat);

  ImagePyramid(ImageType   type,
               ImageFormat format,
               uint8_t*    buffer,
               int         lenght,
               int         width,
               int         height);

  ~ImagePyramid();

protected:
  void        createImagePyrmid();
  static void convertToGray(cv::Mat& src, cv::Mat& dst);

protected:
  cv::Mat              mGray;
  std::vector<cv::Mat> mPyramids;

public:
  cv::Mat               getGray() { return mGray; }
  std::vector<cv::Mat>& getPyramids() { return mPyramids; }
};
};  //namespace db
}  //namespace toy