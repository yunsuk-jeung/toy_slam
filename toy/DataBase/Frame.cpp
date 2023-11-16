#include "ToyLogger.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "Frame.h"

namespace toy {
namespace db {

Frame::Frame(ImagePyramid* imagePyramid)
    : mId{-1}
    , mImagePyramid0{&imagePyramid[0]}
    , mImagePyramid1{&imagePyramid[1]}
    , mFeature0{new Feature()}
    , mFeature1{new Feature()} {}

Frame::~Frame() {
  delete[] mImagePyramid0;
  mImagePyramid0 = nullptr;
  mImagePyramid1 = nullptr;

  delete mFeature0;
  mFeature0 = nullptr;

  delete mFeature1;
  mFeature1 = nullptr;
}

}  //namespace db
}  //namespace toy
