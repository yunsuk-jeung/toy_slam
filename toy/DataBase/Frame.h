#pragma once
#include <memory>

namespace toy {
namespace db {
class ImagePyramid;
class Map;
class Frame {
  friend class Map;

public:

protected:
  Frame();
  ~Frame();

protected:
  ImagePyramid* mImagePyramid;
};
}  //namespace db
}  //namespace toy