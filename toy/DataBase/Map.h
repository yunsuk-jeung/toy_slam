#pragma once
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_set.h>
#include "Frame.h"

namespace toy {
namespace db {
class Frame;
class Landmark;
class Map {
public:
  Map()  = default;
  ~Map() = default;

  virtual Frame* createNewFrame() = 0;

protected:
  tbb::concurrent_set<Frame*> mFrames;
  //tbb::concurrent_set<Landmark*> mLandmarks;
};

}  //namespace db
}  //namespace toy