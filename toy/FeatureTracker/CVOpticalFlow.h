#pragma once
#include "config.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Feature.h"
#include "Frame.h"
#include "PointMatcher.h"
namespace toy {
class CVOpticalFlow : public PointMatcher {
public:
  CVOpticalFlow()  = default;
  ~CVOpticalFlow() = default;

  virtual size_t match(db::Frame* prev, db::Frame* curr) override {
    if (prev == nullptr)
      return size_t(0);

    auto maxIdx = 1;

    if (curr->getImagePyramid(1)->type() == 1) {
      maxIdx = 2;
    }
    size_t trackedCount = 0;

    for (size_t k = 0; k < maxIdx; ++k) {
      auto& pyramid0    = prev->getImagePyramid(k)->getPyramids();
      auto& keyPoints0  = prev->getFeature(k)->getKeypoints();
      auto& ids0        = keyPoints0.mIds;
      auto& levels0     = keyPoints0.mLevels;
      auto& uvs0        = keyPoints0.mUVs;
      auto& trackCount0 = keyPoints0.mTrackCounts;
      auto& undists0    = keyPoints0.mUndists;

      if (ids0.empty())
        return size_t(0);

      auto& pyramid1 = curr->getImagePyramid(k)->getPyramids();

      std::vector<cv::Point2f> uvs;
      std::vector<cv::Point2f> undists;
      const auto patch = cv::Size2i(Config::Vio::patchSize, Config::Vio::patchSize);

      std::vector<uchar> statusO;
      cv::calcOpticalFlowPyrLK(pyramid0,
                               pyramid1,
                               uvs0,
                               uvs,
                               statusO,
                               cv::noArray(),
                               patch,
                               Config::Vio::maxPyramidLevel);

      auto* cam1 = curr->getCamera(0);
      cam1->undistortPoints(uvs, undists);

      std::vector<uchar> statusE;
      cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

      const double threshold = Config::Vio::epipolarThreashold;
      cv::findFundamentalMat(uvs0, uvs, cv::FM_RANSAC, 1.0, 0.99, statusE);
      //cv::findEssentialMat(undists0, undists, I, cv::RANSAC, 0.99, threshold, statusE);

      std::vector<cv::Point2f> reverse_uvs;
      std::vector<uchar>       reverse_status;

      cv::calcOpticalFlowPyrLK(pyramid1,
                               pyramid0,
                               uvs,
                               reverse_uvs,
                               reverse_status,
                               cv::noArray(),
                               patch,
                               Config::Vio::maxPyramidLevel);

      for (size_t i = 0; i < reverse_status.size(); ++i) {
        if (!statusO[i]) {
          continue;
        }

        if (!reverse_status[i]) {
          statusO[i] = false;
          continue;
        }
        //cv::Point2f dist       = uvs0[i] - reverse_uvs[i];
        //float       distNormSq = dist.x * dist.x + dist.y * dist.y;
        //
        //if (distNormSq > 5.0f) {
        //  statusO[i] = 0;
        //}
      }

      auto  trackSize  = statusE.size();
      auto& keyPoints1 = curr->getFeature(k)->getKeypoints();
      keyPoints1.reserve(trackSize);

      auto& ids1        = keyPoints1.mIds;
      auto& levels1     = keyPoints1.mLevels;
      auto& uvs1        = keyPoints1.mUVs;
      auto& trackCount1 = keyPoints1.mTrackCounts;
      auto& undists1    = keyPoints1.mUndists;
      //auto& featureType = keyPoints1.mFeatureType;

      for (size_t i = 0; i < trackSize; ++i) {
        if (statusO[i] == 0 || statusE[i] == 0)
          continue;
        ids1.push_back(ids0[i]);
        levels1.push_back(levels0[i]);
        uvs1.push_back(uvs[i]);
        trackCount1.push_back(++trackCount0[i]);
        undists1.push_back(undists[i]);
        //featureType.push_back(0);
      }
      if (k == 0) {
        trackedCount = ids1.size();
      }
      //#####################################################################
      if (Config::Vio::showMonoTracking && k == 0) {
        cv::Mat image1 = pyramid1[0].clone();
        cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);

        //for (int i = 0; i < uvs0.size(); i++) {
        //  if (statusO[i] == 0 || statusE[i] == 0) {
        //    cv::line(image1, uvs0[i], uvs[i], {0.0, 0.0, 0}, 1);
        //    cv::circle(image1, uvs[i], 4, {0.0, 0.0, 0}, -1);
        //  }
        //}

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
        cv::imshow("mono opticalflow", image1);
        cv::waitKey(1);
      }
    }
    //#####################################################################

    return trackedCount;
  }

  virtual size_t matchStereo(db::Frame*                   frame,
                             std::shared_ptr<db::Feature> detectedFeature) override {
    auto& pyramid0    = frame->getImagePyramid(0)->getPyramids();
    auto& keyPoints0  = detectedFeature->getKeypoints();
    auto& ids0        = keyPoints0.mIds;
    auto& levels0     = keyPoints0.mLevels;
    auto& uvs0        = keyPoints0.mUVs;
    auto& trackCount0 = keyPoints0.mTrackCounts;
    auto& undists0    = keyPoints0.mUndists;

    auto& pyramid1 = frame->getImagePyramid(1)->getPyramids();

    const auto patch = cv::Size2i(Config::Vio::patchSize, Config::Vio::patchSize);

    std::vector<uchar> status;

    std::vector<cv::Point2f> uvs;
    std::vector<cv::Point2f> undists;

    cv::calcOpticalFlowPyrLK(pyramid0,
                             pyramid1,
                             uvs0,
                             uvs,
                             status,
                             cv::noArray(),
                             patch,
                             Config::Vio::maxPyramidLevel);

    auto* cam1 = frame->getCamera(1);
    cam1->undistortPoints(uvs, undists);
    std::vector<uchar> statusE;
    //cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    //const double threshold = Config::Vio::epipolarThreashold;
    //cv::findEssentialMat(undists0, undists1, I, cv::RANSAC, 0.99, threshold, statusE);
    cv::findFundamentalMat(uvs0, uvs, cv::FM_RANSAC, 1.0, 0.99, statusE);

    auto trackSize = uvs0.size();

    auto& keyPoints1  = frame->getFeature(1)->getKeypoints();
    auto& ids1        = keyPoints1.mIds;
    auto& levels1     = keyPoints1.mLevels;
    auto& uvs1        = keyPoints1.mUVs;
    auto& trackCount1 = keyPoints1.mTrackCounts;
    auto& undists1    = keyPoints1.mUndists;
    auto& idToidx     = keyPoints1.mIdIdx;
    auto  prevSize    = keyPoints1.size();
    keyPoints1.reserve(keyPoints1.size() + trackSize);

    size_t stereoFeatureSize = 0u;
    for (int i = 0; i < trackSize; ++i) {
      if (status[i] == 0 || statusE[i] == 0) {
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
        cv::line(image1, uvs0[i], uvs1[i], {0.0, 255.0, 0.0}, 1);
        cv::circle(image1, uvs1[i], 4, {0.0, 255.0, 0.0}, -1);
      }
      for (int i = 0; i < uvs0.size(); i++) {
        if (status[i] == 0 || statusE[i] == 0) {
          cv::line(image1, uvs0[i], uvs1[i], {255.0, 0.0, 0.0}, 1);
          cv::circle(image1, uvs1[i], 2, {255.0, 0.0, 0.0}, -1);
        }
      }
      cv::imshow("stereo opticalflow1", image1);
      cv::waitKey(1);
    }
    //#####################################################################

    return stereoFeatureSize;
  }

protected:
};

}  //namespace toy