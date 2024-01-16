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
                             Config::Vio::pyramidLevel);

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
                             Config::Vio::pyramidLevel);

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

      //if (distNormSq > 5) {
      //statusO[i] = false;
      //}
    }

    auto  trackSize  = statusE.size();
    auto& keyPoints1 = curr->getFeature(0)->getKeypoints();
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

    //#####################################################################
    if (Config::Vio::showMonoTracking) {
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
    //#####################################################################

    return ids1.size();
  }

  virtual size_t matchStereo(db::Frame* frame) override {
    auto& pyramid0    = frame->getImagePyramid(0)->getPyramids();
    auto& keyPoints0  = frame->getFeature(0)->getKeypoints();
    auto& ids0        = keyPoints0.mIds;
    auto& levels0     = keyPoints0.mLevels;
    auto& uvs0        = keyPoints0.mUVs;
    auto& trackCount0 = keyPoints0.mTrackCounts;
    auto& undists0    = keyPoints0.mUndists;

    auto& pyramid1    = frame->getImagePyramid(1)->getPyramids();
    auto& keyPoints1  = frame->getFeature(1)->getKeypoints();
    auto& ids1        = keyPoints1.mIds;
    auto& levels1     = keyPoints1.mLevels;
    auto& uvs1        = keyPoints1.mUVs;
    auto& trackCount1 = keyPoints1.mTrackCounts;
    auto& undists1    = keyPoints1.mUndists;

    const auto patch = cv::Size2i(Config::Vio::patchSize, Config::Vio::patchSize);

    std::vector<uchar> status;

    cv::calcOpticalFlowPyrLK(pyramid0,
                             pyramid1,
                             uvs0,
                             uvs1,
                             status,
                             cv::noArray(),
                             patch,
                             Config::Vio::pyramidLevel);

    auto* cam1 = frame->getCamera(1);
    cam1->undistortPoints(uvs1, undists1);
    std::vector<uchar> statusE;
    cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    const double threshold = Config::Vio::epipolarThreashold;
    //cv::findEssentialMat(undists0, undists1, I, cv::RANSAC, 0.99, threshold, statusE);
    cv::findFundamentalMat(uvs0, uvs1, cv::FM_RANSAC, 1.0, 0.99, statusE);

    auto trackSize = uvs0.size();

    size_t stereoFeatureSize = 0u;
    for (int i = 0; i < trackSize; ++i) {
      if (status[i] == 0 || statusE[i] == 0) {
        ids1.push_back(0);
        continue;
      }
      ids1.push_back(1);  //stereo
      ++stereoFeatureSize;
    }

    //Sophus::SE3d    Sbc0    = Sophus::SE3d::exp(frame->getLbc(0));
    //Sophus::SE3d    Sbc1    = Sophus::SE3d::exp(frame->getLbc(1));
    //Sophus::SE3d    Sc1c0   = Sbc1.inverse() * Sbc0;
    //Eigen::Matrix3d Rc1c0   = Sc1c0.so3().matrix();
    //Eigen::Matrix3d Tc1c0_x = Sophus::SO3d::hat(Sc1c0.translation());
    //Eigen::Matrix3d E       = Rc1c0 * Tc1c0_x;

    //constexpr double epipolarThreashold = 0.0015625;  //1.0 / 640;

    //auto trackedSize = uvs0.size();
    //for (int i = 0; i < trackedSize; i++) {
    //  if (status[i] == 0) continue;
    //  const auto& undist0 = undists0[i];
    //  const auto& undist1 = undists1[i];

    //Eigen::Vector3d nuv0 = Eigen::Vector3d(undist0.x, undist0.y, 1.0);
    //Eigen::Vector3d nuv1 = Eigen::Vector3d(undist0.x, undist0.y, 1.0);

    //double err = nuv1.transpose() * E * nuv0;
    //if (err > epipolarThreashold) status[i] = 0;
    //}

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