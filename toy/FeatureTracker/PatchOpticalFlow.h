#pragma once
#include "config.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Feature.h"
#include "Frame.h"
#include "patch.h"
#include "PointMatcher.h"

namespace toy {
class PatchOpticalFlow : public PointMatcher {
public:
  PatchOpticalFlow()  = default;
  ~PatchOpticalFlow() = default;

  virtual size_t match(db::Frame* prev, db::Frame* curr) override {
    if (prev == nullptr)
      return size_t(0u);

    auto& pyramid0    = prev->getImagePyramid(0)->getPyramids();
    auto& keyPoints0  = prev->getFeature(0)->getKeypoints();
    auto& ids0        = keyPoints0.mIds;
    auto& levels0     = keyPoints0.mLevels;
    auto& uvs0        = keyPoints0.mUVs;
    auto& trackCount0 = keyPoints0.mTrackCounts;
    auto& undists0    = keyPoints0.mUndists;

    if (ids0.empty())
      return size_t(0);

    auto& pyramid1 = curr->getImagePyramid(0)->getPyramids();

    for (auto& uv : uvs0) {
      for (int i = 0; i < Config::Vio::pyramidLevel; ++i) {
        float scale = 1 << i;
        
        Patch p(pyramid0[i << 1], uv);
      }
    }
  }
  virtual size_t matchStereo(db::Frame* frame) override { return 0; }

protected:
};

}  //namespace toy