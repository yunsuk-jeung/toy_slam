#include "config.h"
#include "ToyAssert.h"
#include "Feature.h"
#include "FrameState.h"
#include "MapPoint.h"
#include "LocalMap.h"
#include "Factor.h"

namespace toy {
namespace db {
LocalMap::LocalMap() {}

LocalMap::~LocalMap() {}

void LocalMap::reset() {
  mFrameStates.clear();
  mMapPoints.clear();
  mMapPointCandidates.clear();
}

size_t LocalMap::addFrameState(std::shared_ptr<FrameState> frameState) {
  int frameId = frameState->id();

  mFrameStates.insert({frameId, frameState});

  size_t connected = 0u;
  for (auto& frame : frameState->getFrames()) {
    auto& keyPoints = frame->getFeature()->getKeypoints();
    auto  size      = keyPoints.size();

    for (size_t i = 0; i < size; ++i) {
      int id = keyPoints.mIds[i];

      MapPoint::Ptr mp;
      auto          it = mMapPoints.find(id);

      auto& uv     = keyPoints.mUVs[i];
      auto& undist = keyPoints.mUndists[i];

      if (it != mMapPoints.end()) {
        auto candIt = mMapPointCandidates.find(id);
        if (candIt == mMapPointCandidates.end()) {
          mp = std::make_shared<MapPoint>(id);
          mMapPointCandidates.insert({id, mp});
        }
        else {
          mp = candIt->second;
        }
      }
      else {
        mp = it->second;
        if (mp->status() == db::MapPoint::Status::INITIALING) {
          mp->setState(db::MapPoint::Status::TRACKING);
        }
        mMapPoints.insert({mp->id(), mp});
        ++connected;
      }

      auto factor = ReprojectionFactor(frame.get(),
                                       mp.get(),
                                       {uv.x, uv.y},
                                       {undist.x, undist.y, 1.0});
      mp->addFrameFactor(frame, factor);
      frame->addMapPointFactor(mp, factor);
    }
  }
  return connected;
}

void LocalMap::addMapPoint(std::shared_ptr<MapPoint> mp) {
  assert(mMapPoints.count(mp->id()) == 0);
  mMapPoints.insert({mp->id(), mp});
}

void LocalMap::getCurrentStates(std::vector<FrameState::Ptr>& frames,
                                std::vector<MapPoint::Ptr>&   trackingMapPoints,
                                std::vector<MapPoint::Ptr>&   marginedMapPoints) {
  frames.reserve(mFrameStates.size());
  trackingMapPoints.reserve(mMapPoints.size());
  marginedMapPoints.reserve(mFrameStates.size());

  for (auto& [id, frame] : mFrameStates) {
    frames.push_back(frame);
  }

  for (auto& [id, mp] : mMapPoints) {
    int status = static_cast<int>(mp->status());
    if (status < static_cast<int>(MapPoint::Status::TRACKING)) {
      continue;
    }

    if (mp->status() == MapPoint::Status::MARGINED) {
      marginedMapPoints.push_back(mp);
      continue;
    }

    if (mp->getFrameFactors().size() < 2) {
      continue;
    }

    trackingMapPoints.push_back(mp);
  }
}

void LocalMap::removeFrame(int id) {
  auto it = mFrameStates.find(id);
  TOY_ASSERT(it != mFrameStates.end());

  for (auto& f : it->second->getFrames()) {
    for (auto it = mMapPoints.begin(); it != mMapPoints.end();) {
      bool eraseMapPoint = it->second->eraseFrame(f);
      if (eraseMapPoint) {
        it = mMapPoints.erase(it);
      }
      else {
        ++it;
      }
    }

    for (auto it = mMapPointCandidates.begin(); it != mMapPointCandidates.end();) {
      bool eraseMapPoint = it->second->eraseFrame(f);
      if (eraseMapPoint) {
        it = mMapPointCandidates.erase(it);
      }
      else {
        ++it;
      }
    }
  }

  mFrameStates.erase(it);
}

}  //namespace db
}  //namespace toy