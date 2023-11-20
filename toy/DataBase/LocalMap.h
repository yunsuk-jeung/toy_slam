#pragma once
#include "Map.h"

namespace toy {
namespace db {
class Frame;
class LocalMap : public Map {
public:
  LocalMap();
  ~LocalMap();

protected:
  std::vector<Frame*> mFrames;
};

}  //namespace db
}  //namespace toy