#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include "types.h"
#include "macros.h"

namespace toy {
namespace db {
class ImagePyramid {
public:
  ;
  TOY_SMART_PTR(ImagePyramid);

  ImagePyramid() = default;
  ImagePyramid(const ImageData&);

  ~ImagePyramid();

  //static ImagePyramid* clone(ImagePyramid* src);
  ImagePyramid* clone();

protected:
  ImagePyramid(const ImagePyramid* src);
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

class ImagePyramidSet {
public:
  ImagePyramidSet(ImagePyramid* i0, ImagePyramid* i1) {
    mImagePyramid0 = std::unique_ptr<ImagePyramid>(i0);
    mImagePyramid1 = std::unique_ptr<ImagePyramid>(i1);
  }

  using Ptr = std::shared_ptr<ImagePyramidSet>;

  ImagePyramid::Uni mImagePyramid0;
  ImagePyramid::Uni mImagePyramid1;
};

};  //namespace db
}  //namespace toy