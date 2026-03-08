#include "config.h"
#include "ToyAssert.h"
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
  mMapPointCandidates.clear();
}

size_t LocalMap::addFrame(std::shared_ptr<Frame> frame) {
  int frameId = frame->id();

  mFrames.insert({frameId, frame});
  size_t connected = 0u;

  auto& features = frame->getFeatures();
  for (size_t i = 0; i < features.size(); ++i) {
    auto& keyPoints    = frame->getFeature(i)->getKeypoints();
    auto  keyPointSize = keyPoints.size();

    for (size_t j = 0; j < keyPointSize; ++j) {
      int id = keyPoints.mIds[j];

      MapPoint::Ptr mp;
      auto          it = mMapPoints.find(id);

      if (it == mMapPoints.end()) {
        auto candIt = mMapPointCandidates.find(id);
        if (candIt == mMapPointCandidates.end()) {
          //TOY_ASSERT_MESSAGE(i == 0, " adding mp in sub frame");
          if (i != 0) {
            continue;
          }
          mp = std::make_shared<MapPoint>(id);
          mMapPointCandidates.insert({id, mp});
        }
        else {
          mp = candIt->second;
        }
      }
      else {
        if (i == 0) {
          ++connected;
        }
        mp = it->second;
      }

      auto& uv     = keyPoints.mUVs[j];
      auto& undist = keyPoints.mUndists[j];
      auto  factor = ReprojectionFactor(frame,
                                       i,
                                       mp,
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

void LocalMap::getCurrentStates(std::vector<Frame::Ptr>&    frames,
                                std::vector<MapPoint::Ptr>& trackingMapPoints) {
  frames.reserve(mFrames.size());
  trackingMapPoints.reserve(mMapPoints.size());

  for (auto& [id, frame] : mFrames) {
    frames.push_back(frame);
  }

  for (auto& [id, mp] : mMapPoints) {
    int status = static_cast<int>(mp->status());
    if (status < static_cast<int>(MapPoint::Status::TRACKING)) {
      continue;
    }

    if (mp->frameFactorMap().size() < 2) {
      continue;
    }

    trackingMapPoints.push_back(mp);
  }
}

void LocalMap::removeFrame(int64_t id) {
  TOY_ASSERT(mFrames.count(id) > 0);

  auto& f                  = mFrames[id];
  auto& mapPointFactorMaps = f->mapPointFactorMaps();

  std::forward_list<size_t> marginMapPointIds;

  for (auto it = mMapPoints.begin(); it != mMapPoints.end();) {
    auto eraseMapPoint = it->second->eraseFrame(f);

    if (eraseMapPoint) {
      marginMapPointIds.push_front(it->first);
      it = mMapPoints.erase(it);
    }
    else {
      ++it;
    }
  }

  for (auto it = mMapPointCandidates.begin(); it != mMapPointCandidates.end();) {
    auto eraseMapPoint = it->second->eraseFrame(f);

    if (eraseMapPoint) {
      it = mMapPointCandidates.erase(it);
      marginMapPointIds.push_front(it->first);
    }
    else {
      ++it;
    }
  }

  mFrames.erase(id);

  for (auto& [fId, frame] : mFrames) {
    for (auto& mpId : marginMapPointIds) {
      frame->eraseMapPointFactor(mpId);
    }
  }
}

}  //namespace db
}  //namespace toy