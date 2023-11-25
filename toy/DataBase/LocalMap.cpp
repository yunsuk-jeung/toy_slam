#include "config.h"
#include "ToyLogger.h"
#include "Frame.h"
#include "LocalMap.h"
namespace toy {
namespace db {
LocalMap::LocalMap() {
  mFramePtrs.reserve(Config::Vio::mapFrameSize);
}

LocalMap::~LocalMap() {}

void LocalMap::addFramePtr(FramePtr& in) {
  mFramePtrs.push_back(in);
}

Frame* LocalMap::getLatestFrame() {
  if (mFramePtrs.empty()) return nullptr;
  return mFramePtrs.back().get();
}

}  //namespace db
}  //namespace toy