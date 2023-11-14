#pragma once
#include "Map.h"

namespace toy {
namespace db {
class LocalMap : public Map {
public:
  using Map::createNewFrame;
  LocalMap();
  ~LocalMap();

  Frame* createNewFrame(ImagePyramid*) override;

protected:
  using Map::mFrames;
};

}  //namespace db
}  //namespace toy