#include <opencv2/opencv.hpp>
#include "config.h"
#include "ToyLogger.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Feature.h"
#include "PointTracker.h"

namespace toy {
PointTracker::PointTracker(std::string type)
  : mFeatureId{0}
  , mType{type} {
  if (mType == "Fast.OpticalflowLK") { mFeature2D = cv::FastFeatureDetector::create(); }
}

PointTracker::~PointTracker() {}

size_t PointTracker::process(db::Frame* prevFrame, db::Frame* currFrame) {
  size_t trackedPtSize = track(prevFrame, currFrame);

  if (trackedPtSize > 80) return trackedPtSize;

  auto newKptSize = extract(currFrame);

  if (trackedPtSize + newKptSize == 0) return 0;
  if (currFrame->getImagePyramid(1)->type() != 1) return trackedPtSize;

  db::Feature* feature0 = currFrame->getFeature(0);
  db::Feature* feature1 = currFrame->getFeature(1);
  trackStereo(currFrame);

  return trackedPtSize;
}

size_t PointTracker::extract(db::Frame* frame) {
  cv::Mat&     origin  = frame->getImagePyramid(0)->getOrigin();
  db::Feature* feature = frame->getFeature(0);
  Camera*      cam     = frame->getCamera(0);

  cv::Mat mask = createMask(origin, feature);

  std::vector<cv::Mat>                    subImages;
  std::vector<std::vector<cv::KeyPoint> > keyPointsPerSubImage;
  std::vector<cv::Point2i>                offset;

  devideImage(origin, mask, subImages, offset);
  auto subSize = subImages.size();
  keyPointsPerSubImage.resize(subSize);

  int subId = 0;
  for (auto& sub : subImages) {
    auto& kpts = keyPointsPerSubImage[subId++];
    mFeature2D->detect(sub, kpts);
  }

  std::vector<cv::KeyPoint> keyPoints;
  keyPoints.reserve(subSize);

  for (size_t i = 0; i < subSize; ++i) {
    auto& kpts = keyPointsPerSubImage[i];
    if (kpts.empty()) { continue; }
    std::nth_element(kpts.begin(),
                     kpts.begin(),
                     kpts.end(),
                     [](cv::KeyPoint& kpt1, cv::KeyPoint& kpt2) {
                       return kpt1.response > kpt2.response;
                     });
    auto& kpt = kpts.front();
    kpt.pt.x  = kpt.pt.x + offset[i].x;
    kpt.pt.y  = kpt.pt.y + offset[i].y;
    keyPoints.push_back(kpts.front());
  }

  convertCVKeyPointsToFeature(cam, keyPoints, feature);

  cv::Mat image = origin.clone();
  cv::cvtColor(image, image, CV_GRAY2BGR);
  for (const auto& kpt : keyPoints) { cv::circle(image, kpt.pt, 3, {255, 0, 0}, -1); }
  cv::imshow("keyPoint", image);

  return keyPoints.size();
}

void PointTracker::devideImage(cv::Mat&                  src,
                               cv::Mat&                  mask,
                               std::vector<cv::Mat>&     subs,
                               std::vector<cv::Point2i>& offsets) {
  const int rowGridCount   = Config::Vio::rowGridCount;
  const int colGridCount   = Config::Vio::colGridCount;
  const int totalGridCount = rowGridCount * colGridCount;
  subs.reserve(totalGridCount);
  offsets.reserve(totalGridCount);

  const int startCol = (src.cols % colGridCount) >> 1;
  const int startRow = (src.rows % rowGridCount) >> 1;

  const int gridCols = src.cols / colGridCount;
  const int gridRows = src.rows / rowGridCount;

  for (int r = 0; r < rowGridCount; ++r) {
    for (int c = 0; c < colGridCount; ++c) {
      auto offset = cv::Point2i({startCol + gridCols * c}, {startRow + gridRows * r});

      if (mask.at<uchar>(startRow + gridRows * r, startCol + gridCols * c) == 0) continue;

      cv::Rect roi(offset.x, offset.y, gridCols, gridRows);
      cv::Mat  crop = src(roi);
      subs.push_back(crop);
      offsets.push_back(offset);
    }
  }
}

size_t PointTracker::track(db::Frame* prev, db::Frame* curr) {
  if (prev == nullptr) return size_t(0);

  auto& pyramid0    = prev->getImagePyramid(0)->getPyramids();
  auto& keyPoints0  = prev->getFeature(0)->getKeypoints();
  auto& ids0        = keyPoints0.mIds;
  auto& levels0     = keyPoints0.mLevels;
  auto& uvs0        = keyPoints0.mUVs;
  auto& trackCount0 = keyPoints0.mTrackCounts;
  auto& undists0    = keyPoints0.mUndists;

  if (ids0.empty()) return size_t(0);

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
  cv::findEssentialMat(undists0, undists, I, cv::RANSAC, 0.99, threshold, statusE);

  auto  trackSize  = statusE.size();
  auto& keyPoints1 = curr->getFeature(0)->getKeypoints();
  keyPoints1.reserve(trackSize);

  //#####################################################################
  cv::Mat image1 = pyramid1[0].clone();
  cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < uvs0.size(); i++) {
    if (statusO[i] == 0) {
      cv::line(image1, uvs0[i], uvs[i], {0.0, 0.0, 255.0}, 1);
      cv::circle(image1, uvs[i], 4, {0.0, 0.0, 255.0}, -1);
    }
    else {
      cv::line(image1, uvs0[i], uvs[i], {255, 0.0, 0.0}, 1);
      cv::circle(image1, uvs[i], 4, {255, 0.0, 0.0}, -1);
    }
  }
  //#####################################################################
  auto& ids1        = keyPoints1.mIds;
  auto& levels1     = keyPoints1.mLevels;
  auto& uvs1        = keyPoints1.mUVs;
  auto& trackCount1 = keyPoints1.mTrackCounts;
  auto& undists1    = keyPoints1.mUndists;
  auto& featureType = keyPoints1.mFeatureType;

  for (size_t i = 0; i < trackSize; ++i) {
    if (statusO[i] == 0 || statusE[i] == 0) continue;
    ids1.push_back(ids0[i]);
    levels1.push_back(levels0[i]);
    uvs1.push_back(uvs[i]);
    trackCount1.push_back(++trackCount0[i]);
    undists1.push_back(undists[i]);
    featureType.push_back(0);
  }

  //#####################################################################
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
  //#####################################################################

  return ids1.size();
}

size_t PointTracker::trackStereo(db::Frame* frame) {
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
  //#####################################################################
  cv::Mat image0 = pyramid0[0].clone();
  cv::cvtColor(image0, image0, cv::COLOR_GRAY2BGR);

  cv::Mat image1 = pyramid1[0].clone();
  cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < uvs0.size(); i++) {
    if (status[i] == 0) {
      cv::line(image1, uvs0[i], uvs1[i], {0.0, 0.0, 255.0}, 1);
      cv::circle(image1, uvs1[i], 4, {0.0, 0.0, 255.0}, -1);
    }
    else {
      cv::line(image1, uvs0[i], uvs1[i], {0.0, 255.0, 0.0}, 1);
      cv::circle(image1, uvs1[i], 4, {0.0, 255.0, 0.0}, -1);
    }
    cv::circle(image0, uvs0[i], 4, {0.0, 255.0, 0.0}, -1);
  }
  //#####################################################################

  std::vector<uchar> statusE;
  cv::Mat            I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

  const double threshold = Config::Vio::epipolarThreashold;
  cv::findEssentialMat(undists0, undists1, I, cv::RANSAC, 0.99, threshold, statusE);

  auto trackSize = uvs0.size();

  for (int i = 0; i < trackSize; ++i) {
    if (status[i] == 0 || statusE[i] == 0) {
      ids1.push_back(0);
      continue;
    }
    ids1.push_back(1);  //stereo
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
  for (int i = 0; i < uvs0.size(); i++) {
    if (status[i] == 0 || statusE[i] == 0) {
      cv::line(image1, uvs0[i], uvs1[i], {255.0, 0.0, 0.0}, 1);
      cv::circle(image1, uvs1[i], 2, {255.0, 0.0, 0.0}, -1);
    }
  }
  cv::imshow("stereo opticalflow0", image0);
  cv::imshow("stereo opticalflow1", image1);
  cv::waitKey(1);
  //#####################################################################

  return size_t();
}

void PointTracker::convertCVKeyPointsToFeature(Camera*                    cam,
                                               std::vector<cv::KeyPoint>& kpts,
                                               db::Feature*               feature) {
  db::Feature newFeat;
  auto&       newKpts = feature->getKeypoints();
  newKpts.reserve(kpts.size());

  auto& ids        = newKpts.mIds;
  auto& levels     = newKpts.mLevels;
  auto& points     = newKpts.mUVs;
  auto& trackCount = newKpts.mTrackCounts;
  auto& undists    = newKpts.mUndists;

  for (const auto& kpt : kpts) {
    ids.push_back(mFeatureId++);
    levels.push_back(kpt.octave);
    points.push_back(kpt.pt);
    trackCount.push_back(0);
  }
  //asdfasdf
  cam->undistortPoints(points, undists);

  auto& keypoints = feature->getKeypoints();
  auto  size      = newKpts.size() + keypoints.size();

  keypoints.reserve(size);
  keypoints.push_back(newKpts);
}

cv::Mat PointTracker::createMask(const cv::Mat& origin, db::Feature* feature) {
  cv::Mat mask = cv::Mat(origin.rows, origin.cols, origin.type(), cv::Scalar(255));

  const auto& uvs = feature->getKeypoints().mUVs;

  const int rowGridCount = Config::Vio::rowGridCount;
  const int colGridCount = Config::Vio::colGridCount;

  const int gridCols = origin.cols / colGridCount;
  const int gridRows = origin.rows / rowGridCount;
  const int radius   = gridCols > gridRows ? gridRows >> 1 : gridCols >> 1;

  for (const auto& uv : uvs) { cv::circle(mask, uv, radius, {0}, -1); }
  //#####################################################################
  cv::imshow("mask", mask);
  cv::waitKey(1);
  //#####################################################################

  return mask;
}

}  //namespace toy