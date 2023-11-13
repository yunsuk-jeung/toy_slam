#include "ImagePyramid.h"
#include "Logger.h"
#include "config.h"

namespace toy {
namespace db {
ImagePyramid::ImagePyramid(ImageType type, cv::Mat in) {

  switch (type) {
  case ImageType::MAIN:
    convertToGray(in, origin);
    createImagePyrmid();
    break;
  case ImageType::SUB:
    convertToGray(in, origin);
    break;
  default:
    origin = in;
    break;
  }
}

ImagePyramid::ImagePyramid(ImageType type,
                           uint8_t*  buffer,
                           int       lenght,
                           int       width,
                           int       height) {

  switch (type) {
  case ImageType::MAIN:
    origin = cv::Mat(height, width, CV_8UC1, buffer);
    break;

  default:
    break;
  }
}

ImagePyramid::~ImagePyramid() {

  pyramids.clear();
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