#include "ImagePyramid.h"
#include "ToyLogger.h"
#include "config.h"

namespace toy {
namespace db {
ImagePyramid::ImagePyramid(ImageType type, cv::Mat in) {

  switch (type) {
  case ImageType::MAIN:
  case ImageType::SUB: {

    convertToGray(in, mGray);
    createImagePyrmid();

    mW = mGray.cols;
    mH = mGray.rows;
    mL = mW * mH;

    break;
  }
  default:
    mGray = in;
    break;
  }
}

ImagePyramid::ImagePyramid(ImageType type, int cvFormat, uint8_t* buffer, int w, int h) {
  switch (type) {
  case ImageType::MAIN:
  case ImageType::SUB: {
    cv::Mat in = cv::Mat(h, w, cvFormat, buffer).clone();

    convertToGray(in, mGray);
    createImagePyrmid();

    mW = mGray.cols;
    mH = mGray.rows;
    mL = mW * mH * sizeof(uint8_t);

    break;
  }
  default: {
    cv::Mat in = cv::Mat(h, w, cvFormat, buffer);
    mGray      = in;

    break;
  }
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