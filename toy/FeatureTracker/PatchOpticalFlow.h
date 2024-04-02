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

    size_t trackedCount = 0;

    auto& pyramid0    = prev->getImagePyramid()->getPyramids();
    auto& keyPoints0  = prev->getFeature()->getKeypoints();
    auto& ids0        = keyPoints0.mIds;
    auto& levels0     = keyPoints0.mLevels;
    auto& uvs0        = keyPoints0.mUVs;
    auto& trackCount0 = keyPoints0.mTrackCounts;
    auto& undists0    = keyPoints0.mUndists;

    if (ids0.empty())
      return;

    auto& pyramid1 = curr->getImagePyramid()->getPyramids();

    std::vector<cv::Point2f> uvs = uvs0;
    std::vector<cv::Point2f> undists;
    std::vector<uchar>       statusO;
    const size_t             uvSize = uvs0.size();
    statusO.resize(uvSize, 0);

    //YSTODO tbb
    for (auto i = 0; i < uvSize; ++i) {
      const auto& uv0 = uvs0[i];
      auto&       uv1 = uvs[i];

      bool valid = matchPoint(pyramid0, pyramid1, uv0, uv1);
      if (valid) {
        cv::Point2f recovered;
        valid &= matchPoint(pyramid1, pyramid0, uv1, recovered);
        if (valid) {
          cv::Point2f dist       = uv0 - recovered;
          float       distNormSq = dist.x * dist.x + dist.y * dist.y;
          if (distNormSq < 0.04f) {
            statusO[i] = 1u;
          }
        }
      }

      auto* cam1 = curr->getCamera();
      cam1->undistortPoints(uvs, undists);

      //std::vector<uchar> statusE;
      //cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

      //cv::findFundamentalMat(uvs0, uvs, cv::FM_RANSAC, 1.0, 0.99, statusE);

      auto  trackSize  = statusO.size();
      auto& keyPoints1 = curr->getFeature()->getKeypoints();
      keyPoints1.reserve(trackSize);

      auto& ids1        = keyPoints1.mIds;
      auto& levels1     = keyPoints1.mLevels;
      auto& uvs1        = keyPoints1.mUVs;
      auto& trackCount1 = keyPoints1.mTrackCounts;
      auto& undists1    = keyPoints1.mUndists;
      auto& idToidx     = keyPoints1.mIdIdx;

      size_t trackedIdx = 0;
      for (size_t i = 0; i < trackSize; ++i) {
        if (statusO[i] == 0 /*|| statusE[i] == 0*/)
          continue;
        ids1.push_back(ids0[i]);
        levels1.push_back(levels0[i]);
        uvs1.push_back(uvs[i]);
        trackCount1.push_back(++trackCount0[i]);
        undists1.push_back(undists[i]);
        idToidx[ids0[i]] = trackedIdx++;
      }

      trackedCount = ids1.size();
      if (Config::Vio::showMonoTracking) {
        cv::Mat image0 = pyramid0[0].clone();
        cv::Mat image1 = pyramid1[0].clone();
        cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(image0, image0, cv::COLOR_GRAY2BGR);

        auto calcColor = [](int trackCount) -> cv::Scalar {
          trackCount   = std::max(0, std::min(trackCount, 20));
          double ratio = trackCount / 30.0;
          int    blue  = static_cast<int>((1 - ratio) * 255);
          int    red   = static_cast<int>(ratio * 255);
          return cv::Scalar(blue, 0, red);
        };

        for (int i = 0; i < uvs0.size(); i++) {
          if (statusO[i] == 0 /*|| statusE[i] == 0*/) {
            cv::line(image1, uvs0[i], uvs[i], {0, 0, 255}, 1);
            cv::circle(image1, uvs0[i], 4, {0, 0, 255}, -1);
            cv::circle(image1, uvs[i], 3, {0, 0, 0}, -1);
          }
          else {
            auto color = calcColor(trackCount0[i]);
            cv::circle(image1, uvs0[i], 4, {0, 255, 0}, -1);
            cv::line(image1, uvs0[i], uvs[i], color, 1);
            cv::circle(image1, uvs[i], 3, color, -1);
          }
        }

        cv::imshow("mono opticalflow" + std::to_string(curr->Idx()), image1);
        cv::waitKey(1);
      }
    }

    return trackedCount;
  }

  virtual size_t matchStereo(db::Frame*                   src,
                             db::Frame*                   dst,
                             std::shared_ptr<db::Feature> detectedFeature) override {
    auto& pyramid0    = src->getImagePyramid()->getPyramids();
    auto& keyPoints0  = detectedFeature->getKeypoints();
    auto& ids0        = keyPoints0.mIds;
    auto& levels0     = keyPoints0.mLevels;
    auto& uvs0        = keyPoints0.mUVs;
    auto& trackCount0 = keyPoints0.mTrackCounts;
    auto& undists0    = keyPoints0.mUndists;

    auto& pyramid1 = dst->getImagePyramid()->getPyramids();

    std::vector<cv::Point2f> uvs = uvs0;
    std::vector<cv::Point2f> undists;
    std::vector<uchar>       statusO;
    const size_t             uvSize = uvs0.size();
    statusO.resize(uvSize, 0);

    //YSTODO tbb
    for (auto i = 0; i < uvSize; ++i) {
      const auto& uv0 = uvs0[i];
      auto&       uv1 = uvs[i];

      bool valid = matchPoint(pyramid0, pyramid1, uv0, uv1);
      if (valid) {
        cv::Point2f recovered;
        valid &= matchPoint(pyramid1, pyramid0, uv1, recovered);
        if (valid) {
          cv::Point2f dist       = uv0 - recovered;
          float       distNormSq = dist.x * dist.x + dist.y * dist.y;
          if (distNormSq < 0.04f) {
            statusO[i] = 1u;
          }
        }
      }
    }

    auto* cam1 = dst->getCamera();
    cam1->undistortPoints(uvs, undists);

    auto& keyPoints1 = dst->getFeature()->getKeypoints();

    auto trackSize = statusO.size();
    auto prevSize  = keyPoints1.size();

    keyPoints1.reserve(keyPoints1.size() + trackSize);

    auto& ids1        = keyPoints1.mIds;
    auto& levels1     = keyPoints1.mLevels;
    auto& uvs1        = keyPoints1.mUVs;
    auto& trackCount1 = keyPoints1.mTrackCounts;
    auto& undists1    = keyPoints1.mUndists;
    auto& idToidx     = keyPoints1.mIdIdx;

    size_t stereoFeatureSize = 0u;
    for (int i = 0; i < trackSize; ++i) {
      if (statusO[i] == 0 /* || statusE[i] == 0*/) {
        continue;
      }

      ids1.push_back(ids0[i]);  //stereo
      levels1.push_back(levels0[i]);
      uvs1.push_back(uvs[i]);
      trackCount1.push_back(trackCount0[i]);
      undists1.push_back(undists[i]);
      idToidx[ids0[i]] = prevSize + stereoFeatureSize;
      ++stereoFeatureSize;
    }

    //#####################################################################
    if (Config::Vio::showStereoTracking) {
      cv::Mat image1 = pyramid1[0].clone();
      cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);

      for (int i = 0; i < uvs0.size(); i++) {
        if (idToidx.count(ids0[i]) < 1)
          continue;
        auto idx1 = idToidx[ids0[i]];
        cv::line(image1, uvs0[i], uvs1[idx1], {0.0, 255.0, 0.0}, 1);
        cv::circle(image1, uvs1[idx1], 4, {0.0, 255.0, 0.0}, -1);
      }
      for (int i = 0; i < uvs0.size(); i++) {
        if (idToidx.count(ids0[i]) < 1)
          continue;

        if (statusO[i] == 0 /*|| statusE[i] == 0*/) {
          auto idx1 = idToidx[ids0[i]];
          cv::line(image1, uvs0[i], uvs1[idx1], {255.0, 0.0, 0.0}, 1);
          cv::circle(image1, uvs1[idx1], 2, {255.0, 0.0, 0.0}, -1);
        }
      }
      cv::imshow("stereo opticalflow1", image1);
      cv::waitKey(1);
    }
    //#####################################################################

    return stereoFeatureSize;
  }

protected:
  bool matchPoint(std::vector<cv::Mat>& srcs,
                  std::vector<cv::Mat>& dsts,
                  const cv::Point2f&    uv0,
                  cv::Point2f&          uv1) {
    auto pyrLevel = (srcs.size() >> 1) - 1;
    bool valid    = true;
    uv1           = uv0;

    for (int i = pyrLevel; valid && i >= 0; --i) {
      float scale = 1 << i;

      size_t idx = i << 1;
      Patch  p(srcs[idx], uv0 / scale);
      uv1 /= scale;

      valid &= p.isValid();

      if (!valid) {
        continue;
      }

      valid &= p.match(dsts[idx], uv1);
      uv1 *= scale;
    }
    return valid;
  }
};

}  //namespace toy