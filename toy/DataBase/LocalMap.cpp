#include "config.h"
#include "ToyLogger.h"
#include "Frame.h"
#include "LocalMap.h"
namespace toy {
namespace db {
LocalMap::LocalMap() {
  mFrames.reserve(Config::Vio::mapFrameSize);
}

LocalMap::~LocalMap() {}

}  //namespace db
}  //namespace toy