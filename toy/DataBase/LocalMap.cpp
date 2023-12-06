#include "config.h"
#include "ToyLogger.h"
#include "Feature.h"
#include "Frame.h"
#include "MapPoint.h"
#include "LocalMap.h"
#include "Factor.h"

namespace toy {
namespace db {
LocalMap::LocalMap() {}

LocalMap::~LocalMap() {}

void LocalMap::reset() {
  mFrames.clear();
  mMapPoints.clear();
}

void LocalMap::addFrame(std::shared_ptr<Frame> frame) {
  mFrames.insert({frame->id(), frame});
  auto& keyPoints0 = frame->getFeature(0)->getKeypoints();
  auto& keyPoints1 = frame->getFeature(1)->getKeypoints();

  auto size0 = keyPoints0.size();
  auto size1 = keyPoints1.size();

  for (size_t i = 0; i < size0; ++i) {
    int id = keyPoints0.mIds[i];

    MapPoint::Ptr mp;
    auto          it = mMapPoints.find(id);

    if (it == mMapPoints.end())
      mp = std::make_shared<MapPoint>(id);
    else
      mp = it->second;

    mMapPoints.insert({mp->id(), mp});
    auto& uv0     = keyPoints0.mUVs[i];
    auto& undist0 = keyPoints0.mUndists[i];

    auto factor = ReprojectionFactor(frame.get(),
                                     mp.get(),
                                     {uv0.x, uv0.y},
                                     {undist0.x, undist0.y, 1.0});

    if (size1 > 0) {
      if (keyPoints1.mIds[i] == 1) {
        auto& uv1     = keyPoints1.mUVs[i];
        auto& undist1 = keyPoints1.mUndists[i];
        factor.addStereo({uv1.x, uv1.y}, {undist1.x, undist1.y, 1.0});
      }
    }

    frame->addMapPointFactor(mp, factor);
    mp->addFrameFactor(frame, factor);
  }
}

void LocalMap::getCurrentStates(std::vector<Frame::Ptr>&    frames,
                                std::vector<MapPoint::Ptr>& mapPoints) {
  for (auto& [id, frame] : mFrames) { frames.push_back(frame); }
  for (auto& [id, mp] : mMapPoints) {
    if (static_cast<int>(mp->status()) < static_cast<int>(MapPoint::Status::TRACKING)) {
      continue;
    }
    mapPoints.push_back(mp);
  }
}

}  //namespace db
}  //namespace toy