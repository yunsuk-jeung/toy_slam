#include "config.h"
#include "ToyLogger.h"
#include "Feature.h"
#include "Frame.h"
#include "MapPoint.h"
#include "LocalMap.h"

namespace toy {
namespace db {
LocalMap::LocalMap() {}

LocalMap::~LocalMap() {}

void LocalMap::reset() {
  mFrames.clear();
  mMapPoints.clear();
}

void LocalMap::addFrame(std::shared_ptr<Frame> frame) {
  mFrames.insert({frame->Id(), frame});
  auto& keyPoints0 = frame->getFeature(0)->getKeypoints();

  auto size = keyPoints0.size();

  for (size_t i = 0; i < size; ++i) {
    if (mMapPoints.find(keyPoints0.mIds[i]) != mMapPoints.end()) continue;

    MapPoint::Ptr mp = MapPoint::Ptr(new MapPoint());
    mMapPoints.insert({mp->Id(), mp});
  }
}

Frame::Ptr LocalMap::getLatestFrame() {
  if (mFrames.empty()) return nullptr;
  return mFrames.rbegin()->second;
}

}  //namespace db
}  //namespace toy