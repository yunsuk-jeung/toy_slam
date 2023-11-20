#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include "types.h"

namespace toy {
namespace db {
class ImagePyramid {
public:
  ImagePyramid() = delete;
  ImagePyramid(const ImageData&);
  //ImagePyramid(ImageType type, cv::Mat mat);
  //ImagePyramid(ImageType type, int format, uint8_t* buffer, int width, int
  //height);

  ~ImagePyramid();

protected:
  void        createImagePyrmid();
  static void convertToGray(cv::Mat& src, cv::Mat& dst);

protected:
  int                  mType;
  cv::Mat              mOrigin;
  int                  mW;
  int                  mH;
  int                  mL;
  std::vector<cv::Mat> mPyramids;

public:
  int                   type() { return mType; }
  cv::Mat&              getOrigin() { return mOrigin; }
  std::vector<cv::Mat>& getPyramids() { return mPyramids; }
};
};  //namespace db
}  //namespace toy