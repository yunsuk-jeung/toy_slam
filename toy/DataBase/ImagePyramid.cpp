#include "ImagePyramid.h"
#include "Logger.h"

namespace toy {
namespace db {
ImagePyramid::ImagePyramid(ImageType type, cv::Mat in) {
  switch (type) {
  case ImageType::MAIN:
    if (in.type() != 0)
      in.convertTo(origin, CV_8UC1);
    else
      origin = in;
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

};  //namespace db
}  //namespace toy