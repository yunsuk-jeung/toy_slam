#pragma once
//#include <tbb/concorent_>
#include <tbb/concurrent_unordered_map.h>
#include "Frame.h"

namespace toy {
namespace db {
template <typename T>
class Frame;
template <typename FLOAT>
class Map {
public:
  Map()  = default;
  ~Map() = default;

  Frame<FLOAT>* createNewFrame() { return new Frame<FLOAT>(); }

protected:
};

}  //namespace db
}  //namespace toy