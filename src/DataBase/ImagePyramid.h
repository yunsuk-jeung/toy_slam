#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include "types.h"
#include "macros.h"

namespace toy {
namespace db {
class ImagePyramid {
public:
  USING_SMART_PTR(ImagePyramid);

  ImagePyramid() = default;
  ImagePyramid(const ImageData&);
  ImagePyramid(const ImagePyramid* src);

  ~ImagePyramid();

  //static ImagePyramid* clone(ImagePyramid* src);
  ImagePyramid::Ptr clone();

protected:
  void        createImagePyrmid();
  static void convertToGray(cv::Mat& src, cv::Mat& dst);

protected:
  int                       mType;
  cv::Mat                   mOrigin;
  int                       mW;
  int                       mH;
  int                       mL;
  std::vector<cv::Mat>      mPyramids;
  static cv::Ptr<cv::CLAHE> clahe;

public:
  int                   type() { return mType; }
  cv::Mat&              getOrigin() { return mOrigin; }
  std::vector<cv::Mat>& getPyramids() { return mPyramids; }
};

class ImagePyramidSet {
public:
  using Ptr = std::shared_ptr<ImagePyramidSet>;
  ImagePyramidSet(std::vector<ImagePyramid::Ptr>& images) { images_.swap(images); }
  std::vector<ImagePyramid::Ptr> images_;
};

};  //namespace db
}  //namespace toy