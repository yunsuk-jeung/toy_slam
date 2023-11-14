#pragma once
#include "Map.h"

namespace toy {
namespace db {
class LocalMap : public Map {
public:

protected:
  using Map::mFrames;
}
}  //namespace db
}  //namespace toy