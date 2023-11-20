#include <opencv2/opencv.hpp>
#include "ImagePyramid.h"
#include "ToyLogger.h"
#include "config.h"
namespace toy {
namespace db {

ImagePyramid::ImagePyramid(const ImageData& imageData)
  : mType{imageData.type}
  , mW{0}
  , mH{0}
  , mL{0} {

  switch (mType) {
  case ImageType::NONE:
    break;
  case ImageType::MAIN:
  case ImageType::SUB: {
    cv::Mat in =
      cv::Mat(imageData.h, imageData.w, imageData.format, imageData.buffer).clone();

    convertToGray(in, mOrigin);
    createImagePyrmid();

    mW = mOrigin.cols;
    mH = mOrigin.rows;
    mL = mW * mH * sizeof(uint8_t);

    break;
  }
  default: {
    cv::Mat in = cv::Mat(imageData.h, imageData.w, imageData.format, imageData.buffer);
    mOrigin    = in;
    break;
  }
  }
}

ImagePyramid::~ImagePyramid() {
  mPyramids.clear();
}

void ImagePyramid::createImagePyrmid() {
  const cv::Point2i patch(Config::Vio::patchSize, Config::Vio::patchSize);
  const int&        level = Config::Vio::pyramidLevel;
  cv::buildOpticalFlowPyramid(mOrigin,
                              mPyramids,
                              patch,
                              level,
                              true,
                              cv::BORDER_REFLECT_101,
                              cv::BORDER_CONSTANT);
}

void ImagePyramid::convertToGray(cv::Mat& src, cv::Mat& dst) {
  if (src.type() != 0)
    src.convertTo(dst, CV_8UC1);
  else
    dst = src;
}

};  //namespace db
}  //namespace toy