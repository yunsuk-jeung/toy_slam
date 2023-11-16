#pragma once
#include <memory>

namespace toy {
namespace db {
class ImagePyramid;
class LocalMap;
class Feature;
class Frame {
public:
  friend class MemoryPointerPool;

protected:
  Frame() = delete;
  Frame(ImagePyramid*);
  ~Frame();

protected:
  int           mId;
  ImagePyramid* mImagePyramid0;
  ImagePyramid* mImagePyramid1;

  Feature* mFeature0;
  Feature* mFeature1;

public:
  ImagePyramid* getImagePyramid0() { return mImagePyramid0; }
  ImagePyramid* getImagePyramid1() { return mImagePyramid1; }
  Feature*      getFeature0() { return mFeature0; }
  Feature*      getFeature1() { return mFeature1; }
};
}  //namespace db
}  //namespace toy