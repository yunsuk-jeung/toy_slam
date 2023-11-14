#include "Frame.h"
#include "ImagePyramid.h"

namespace toy {
namespace db {

Frame::Frame(ImagePyramid* imagePyramid) : mImagePyramid{imagePyramid} {}

Frame::~Frame() {
  delete mImagePyramid;
  mImagePyramid = nullptr;
}

}  //namespace db
}  //namespace toy
