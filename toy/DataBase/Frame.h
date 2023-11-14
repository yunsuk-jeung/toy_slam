#pragma once
#include <memory>

namespace toy {
namespace db {
class ImagePyramid;
class LocalMap;
class Frame {
  friend class LocalMap;

public:

protected:
  Frame() = delete;
  Frame(ImagePyramid*);
  ~Frame();

protected:
  ImagePyramid* mImagePyramid;
};
}  //namespace db
}  //namespace toy