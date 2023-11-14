#include "ImagePyramid.h"
#include "Logger.h"
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

ImagePyramid::ImagePyramid(ImageType   type,
                           ImageFormat format,
                           uint8_t*    buffer,
                           int         l,
                           int         w,
                           int         h) {
  switch (type) {
  case ImageType::MAIN:
  case ImageType::SUB: {
    int     cvFormat = requestCVFormat(format);
    cv::Mat in       = cv::Mat(h, w, cvFormat, buffer);

    convertToGray(in, mGray);
    createImagePyrmid();

    mW = mGray.cols;
    mH = mGray.rows;
    mL = mW * mH * sizeof(uint8_t);

    break;
  }
  default: {
    int     cvFormat = requestCVFormat(format);
    cv::Mat in       = cv::Mat(h, w, cvFormat, buffer);
    mGray            = in;

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

int ImagePyramid::requestCVFormat(ImageFormat format) {
  int cvFormat{0};
  switch (format) {
  case ImageFormat::RGB888:
    cvFormat = CV_8UC4;
  default:
    break;
  }
  return cvFormat;
}

};  //namespace db
}  //namespace toy