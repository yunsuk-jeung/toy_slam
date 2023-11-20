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

void LocalMap::addFrame(Frame* in) {
  mFrames.push_back(in);
}

Frame* LocalMap::getLatestFrame() {
  if (mFrames.empty()) return nullptr;
  return mFrames.back();
}

}  //namespace db
}  //namespace toy