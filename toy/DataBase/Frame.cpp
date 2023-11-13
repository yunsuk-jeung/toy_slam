#include "Frame.h"
#include "ImagePyramid.h"

namespace toy {
namespace db {

Frame::Frame() : mImagePyramid{nullptr} {}

Frame::~Frame() {
  delete mImagePyramid;
  mImagePyramid = nullptr;
}

}  //namespace db
}  //namespace toy
