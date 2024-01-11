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
}

bool LocalMap::addFrame(std::shared_ptr<Frame> frame) {
  int frameId = frame->id();

  mFrames.insert({frameId, frame});
  auto& keyPoints0 = frame->getFeature(0)->getKeypoints();
  auto& keyPoints1 = frame->getFeature(1)->getKeypoints();

  auto size0 = keyPoints0.size();
  auto size1 = keyPoints1.size();

  auto newMpCount = 0;
  for (size_t i = 0; i < size0; ++i) {
    int id = keyPoints0.mIds[i];

    MapPoint::Ptr mp;
    auto          it = mMapPoints.find(id);

    auto& uv0     = keyPoints0.mUVs[i];
    auto& undist0 = keyPoints0.mUndists[i];

    if (it == mMapPoints.end()) {
      mp = std::make_shared<MapPoint>(id, frameId);
      mp->setUndist({undist0.x, undist0.y});
      ++newMpCount;
    }
    else {
      mp = it->second;
      if (mp->status() == db::MapPoint::Status::WAITING) {
        mp->setState(db::MapPoint::Status::TRACKING);
      }
    }

    mMapPoints.insert({mp->id(), mp});

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

  if (newMpCount > 0) {
    DEBUG_POINT();
    ToyLogE("created new mp : {}", newMpCount);
  }

  return newMpCount > 0;
}

void LocalMap::getCurrentStates(std::vector<Frame::Ptr>&    frames,
                                std::vector<MapPoint::Ptr>& mapPoints) {
  for (auto& [id, frame] : mFrames) {
    frames.push_back(frame);
  }
  for (auto& [id, mp] : mMapPoints) {
    int status = static_cast<int>(mp->status());
    if (status < static_cast<int>(MapPoint::Status::TRACKING)) {
      continue;
    }
    if (mp->getFrameFactors().size() < 2) {
      continue;
    }

    mapPoints.push_back(mp);
  }
}

void LocalMap::removeFrame(int id) {
  auto it = mFrames.find(id);
  TOY_ASSERT(it != mFrames.end());

  Frame::Ptr& f = it->second;
  for (auto it = mMapPoints.begin(); it != mMapPoints.end();) {
    bool eraseMapPoint = it->second->eraseFrame(f);
    if (eraseMapPoint) {
      it = mMapPoints.erase(it);
    }
    else {
      ++it;
    }
  }

  mFrames.erase(it);
}

}  //namespace db
}  //namespace toy