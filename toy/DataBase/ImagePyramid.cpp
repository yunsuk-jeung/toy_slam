#include "ImagePyramid.h"
#include "Logger.h"
#include "config.h"

namespace toy {
namespace db {
ImagePyramid::ImagePyramid(ImageType type, cv::Mat in) {

  switch (type) {
  case ImageType::MAIN:
    convertToGray(in, mGray);
    createImagePyrmid();
    break;
  case ImageType::SUB:
    convertToGray(in, mGray);
    break;
  default:
    mGray = in;
    break;
  }
}

ImagePyramid::ImagePyramid(ImageType   type,
                           ImageFormat format,
                           uint8_t*    buffer,
                           int         lenght,
                           int         width,
                           int         height) {

  switch (type) {
  case ImageType::MAIN:
    break;

  default:
    break;
  }
}

ImagePyramid::~ImagePyramid() {

  mPyramids.clear();
}

void ImagePyramid::createImagePyrmid() {}

void ImagePyramid::convertToGray(cv::Mat& src, cv::Mat& dst) {
  if (src.type() != 0)
    src.convertTo(dst, CV_8UC1);
  else
    dst = src;
}

};  //namespace db
}  //namespace toy