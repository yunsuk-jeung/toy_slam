#include "config.h"
#include "ToyLogger.h"
#include "Feature.h"
#include "MemoryPointerPool.h"
#include "LocalMap.h"
namespace toy {
namespace db {
LocalMap::LocalMap() {
  mFramePtrs.reserve(Config::Vio::mapFrameSize);
}

LocalMap::~LocalMap() {}

void LocalMap::reset() {
  for (auto& framePtr : mFramePtrs) { framePtr.release(); }
  mFramePtrs.clear();
}

void LocalMap::addFramePtr(FramePtr& in) {
  db::Frame* af = in.get();

  mFramePtrs.push_back(in);
  auto& keyPoints0 = in->getFeature(0)->getKeypoints();

  auto size = keyPoints0.size();

  for (size_t i = 0; i < size; ++i) {
    if (mMapPointPtrs.find(keyPoints0.mIds[i]) != mMapPointPtrs.end()) continue;
    MapPoint*   mp = MemoryPointerPool::createMapPoint();
    MapPointPtr mpPtr(mp);

    mMapPointPtrs.insert({mp->Id(), mpPtr});
  }
}

Frame* LocalMap::getLatestFrame() {
  if (mFramePtrs.empty()) return nullptr;
  return mFramePtrs.back().get();
}

}  //namespace db
}  //namespace toy