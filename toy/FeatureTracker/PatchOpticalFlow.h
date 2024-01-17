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

    std::vector<cv::Point2f> uvs = uvs0;
    std::vector<cv::Point2f> undists;
    std::vector<uchar>       statusO;
    const size_t             uvSize = uvs0.size();
    statusO.resize(uvSize, 0);

    //YSTODO tbb
    for (auto i = 0; i < uvSize; ++i) {
      auto& uv0 = uvs0[i];
      auto& uv1 = uvs[i];

      ToyLogD("before {} {} ___ {} {} ", uv0.x, uv0.y, uv1.x, uv1.y);
      bool valid = matchPoint(pyramid0, pyramid1, uv0, uv1);
      if (!valid) {
        continue;
      }
      ToyLogD("after {} {} ___ {} {} ", uv0.x, uv0.y, uv1.x, uv1.y);

      //cv::Point2f recovered;
      //valid &= matchPoint(pyramid1, pyramid0, uv1, recovered);

      if (!valid) {
        continue;
      }
      //cv::Point2f dist       = uv0 - recovered;
      //float       distNormSq = dist.x * dist.x + dist.y * dist.y;

      //if (distNormSq > 10.0f) {
      //  continue;
      //}
      statusO[i] = 1;
    }

    auto* cam1 = curr->getCamera(0);
    cam1->undistortPoints(uvs, undists);

    std::vector<uchar> statusE;
    cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    const double threshold = Config::Vio::epipolarThreashold;
    cv::findFundamentalMat(uvs0, uvs, cv::FM_RANSAC, 1.0, 0.99, statusE);

    auto  trackSize  = statusE.size();
    auto& keyPoints1 = curr->getFeature(0)->getKeypoints();
    keyPoints1.reserve(trackSize);

    auto& ids1        = keyPoints1.mIds;
    auto& levels1     = keyPoints1.mLevels;
    auto& uvs1        = keyPoints1.mUVs;
    auto& trackCount1 = keyPoints1.mTrackCounts;
    auto& undists1    = keyPoints1.mUndists;

    for (size_t i = 0; i < trackSize; ++i) {
      if (statusO[i] == 0 || statusE[i] == 0)
        continue;
      ids1.push_back(ids0[i]);
      levels1.push_back(levels0[i]);
      uvs1.push_back(uvs[i]);
      trackCount1.push_back(++trackCount0[i]);
      undists1.push_back(undists[i]);
    }
    if (true) {
      cv::Mat image0 = pyramid0[0].clone();
      cv::Mat image1 = pyramid1[0].clone();
      cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);
      cv::cvtColor(image0, image0, cv::COLOR_GRAY2BGR);

      for (int i = 0; i < uvs0.size(); i++) {
        if (statusO[i] == 0 || statusE[i] == 0) {
          cv::line(image1, uvs0[i], uvs[i], {0.0, 0.0, 0}, 1);
          cv::circle(image1, uvs[i], 4, {0.0, 0.0, 0}, -1);
        }
      }

      auto calcColor = [](int trackCount) -> cv::Scalar {
        trackCount   = std::max(0, std::min(trackCount, 20));
        double ratio = trackCount / 30.0;
        int    blue  = static_cast<int>((1 - ratio) * 255);
        int    red   = static_cast<int>(ratio * 255);
        return cv::Scalar(blue, 0, red);
      };
      int iiii = 0;
      for (const auto& uv : uvs1) {
        auto color = calcColor(trackCount1[iiii++]);
        cv::circle(image1, uv, 3, color, -1);
      }
      for (const auto& uv : uvs0) {
        cv::circle(image0, uv, 3, {255, 0, 0}, -1);
      }
      cv::imshow("mono opticalflow0", image0);
      cv::imshow("mono opticalflow1", image1);
      cv::waitKey();
    }
    return ids1.size();
  }
  virtual size_t matchStereo(db::Frame* frame) override { return 0; }

protected:
  bool matchPoint(std::vector<cv::Mat>& srcs,
                  std::vector<cv::Mat>& dsts,
                  const cv::Point2f&    uv0,
                  cv::Point2f&          uv1) {
    auto pyrLevel = (srcs.size() >> 1) - 1;
    bool valid    = true;
    uv1           = uv0;

    for (int i = 0; valid && i >= 0; --i) {
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