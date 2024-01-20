#include <opencv2/opencv.hpp>
#include "config.h"
#include "ToyLogger.h"
#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Feature.h"
#include "PointMatcher.h"
#include "PointTracker.h"

namespace toy {
PointTracker::PointTracker(std::string type)
  : mFeatureId{0u}
  , mStereoTrackingIntervalCount{Config::Vio::stereoTrackingInterval} {
  size_t pos   = type.find(".");
  mFeatureType = type.substr(0, pos);
  mMatcherType = type.substr(pos + 1);

  if (mFeatureType == "Fast") {
    mPointDetector = cv::FastFeatureDetector::create();
  }

  mPointMatcher = PointMatcherFactory::create(mMatcherType);

  //mMaxFeatureSize = Config::Vio::rowGridCount * Config::Vio::colGridCount
  //                  * Config::Vio::minTrackedRatio * 2.0f;

  mGridStatus.resize(Config::Vio::rowGridCount * Config::Vio::colGridCount);
}

PointTracker::~PointTracker() {}

size_t PointTracker::process(db::Frame* prevFrame, db::Frame* currFrame) {
  size_t trackedPtSize = match(prevFrame, currFrame);
  //size_t prevSize      = mPrevIds.size() + 1;
  //size_t newPt         = 0u;

  //int   found  = 0;
  //auto& kptIds = currFrame->getFeature(0)->getKeypoints().mIds;
  //for (auto& id : kptIds) {
  //  if (mPrevIds.count(id)) {
  //    ++found;
  //  }
  //}
  //if (prevSize > mMaxFeatureSize) {
  //  prevSize = mMaxFeatureSize;
  //}

  //float ratio = float(found) / float(prevSize);

  //bool bTrackStereo = false;

  //if (mStereoTrackingIntervalCount == Config::Vio::stereoTrackingInterval) {
  //  mStereoTrackingIntervalCount = 0;
  //  bTrackStereo                 = true;
  //}
  //else {
  //  ++mStereoTrackingIntervalCount;
  //}

  //ToyLogD("currFrame id {} , ratio : {} = {} /{}  ",
  //        currFrame->id(),
  //        ratio,
  //        trackedPtSize,
  //        prevSize);

  //if (ratio < Config::Vio::minTrackedRatio
  //    || trackedPtSize < Config::Vio::minTrackedPoint) {
  //ToyLogD("currFrame id {} , extracting new Feature : {} = {} /{}  ",
  //        currFrame->id(),
  //        ratio,
  //        trackedPtSize,
  //        prevSize);
  size_t newPt = detect(currFrame);
  //mPrevIds.clear();

  //for (auto& id : kptIds) {
  //  mPrevIds.insert(id);
  //}
  //bTrackStereo = true;
  //}

  if (currFrame->getImagePyramid(1)->type() == 1) {
    size_t stereo = matchStereo(currFrame);
  }

  return trackedPtSize;
}

size_t PointTracker::detect(db::Frame* frame) {
  cv::Mat&     origin  = frame->getImagePyramid(0)->getOrigin();
  db::Feature* feature = frame->getFeature(0);
  Camera*      cam     = frame->getCamera(0);

  //cv::Mat mask = createMask(origin, feature);
  checkEmptyGrid(origin, feature);

  std::vector<cv::Mat>                    subImages;
  std::vector<std::vector<cv::KeyPoint> > keyPointsPerSubImage;
  std::vector<cv::Point2i>                offset;
  devideImage(origin, subImages, offset);
  //devideImage(origin, mask, subImages, offset);
  auto subSize = subImages.size();
  keyPointsPerSubImage.resize(subSize);

  int subId = 0;
  for (auto& sub : subImages) {
    auto& kpts = keyPointsPerSubImage[subId++];
    mPointDetector->detect(sub, kpts);
  }

  std::vector<cv::KeyPoint> keyPoints;
  keyPoints.reserve(subSize);

  for (size_t i = 0; i < subSize; ++i) {
    auto& kpts = keyPointsPerSubImage[i];
    if (kpts.empty()) {
      continue;
    }
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

  if (Config::Vio::showExtraction) {
    cv::Mat image = origin.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& kpt : keyPoints) {
      cv::circle(image, kpt.pt, 3, {255, 0, 0}, -1);
    }
    cv::imshow("keyPoint", image);
    cv::waitKey(1);
  }

  return keyPoints.size();
}

void PointTracker::devideImage(cv::Mat&                  src,
                               std::vector<cv::Mat>&     subs,
                               std::vector<cv::Point2i>& offsets) {
  const int& rowGridCount   = Config::Vio::rowGridCount;
  const int& colGridCount   = Config::Vio::colGridCount;
  const int  totalGridCount = rowGridCount * colGridCount;
  subs.reserve(totalGridCount);
  offsets.reserve(totalGridCount);

  const int startCol = (src.cols % colGridCount) >> 1;
  const int startRow = (src.rows % rowGridCount) >> 1;

  const int gridCols = src.cols / colGridCount;
  const int gridRows = src.rows / rowGridCount;

  for (int r = 0; r < rowGridCount; ++r) {
    for (int c = 0; c < colGridCount; ++c) {
      int idx = r * colGridCount + c;

      if (mGridStatus[idx] == 1) {
        continue;
      }
      auto     offset = cv::Point2i({startCol + gridCols * c}, {startRow + gridRows * r});
      cv::Rect roi(offset.x, offset.y, gridCols, gridRows);
      cv::Mat  crop = src(roi);
      subs.push_back(crop);
      offsets.push_back(offset);
    }
  }
}

void PointTracker::devideImage(cv::Mat&                  src,
                               cv::Mat&                  mask,
                               std::vector<cv::Mat>&     subs,
                               std::vector<cv::Point2i>& offsets) {
  const int& rowGridCount   = Config::Vio::rowGridCount;
  const int& colGridCount   = Config::Vio::colGridCount;
  const int  totalGridCount = rowGridCount * colGridCount;
  subs.reserve(totalGridCount);
  offsets.reserve(totalGridCount);

  const int startCol = (src.cols % colGridCount) >> 1;
  const int startRow = (src.rows % rowGridCount) >> 1;

  const int gridCols = src.cols / colGridCount;
  const int gridRows = src.rows / rowGridCount;

  for (int r = 0; r < rowGridCount; ++r) {
    for (int c = 0; c < colGridCount; ++c) {
      if (mask.at<uchar>(startRow + gridRows * r, startCol + gridCols * c) == 0)
        continue;

      auto     offset = cv::Point2i({startCol + gridCols * c}, {startRow + gridRows * r});
      cv::Rect roi(offset.x, offset.y, gridCols, gridRows);
      cv::Mat  crop = src(roi);
      subs.push_back(crop);
      offsets.push_back(offset);
    }
  }
}

size_t PointTracker::match(db::Frame* prev, db::Frame* curr) {
  return mPointMatcher->match(prev, curr);
}

size_t PointTracker::matchStereo(db::Frame* frame) {
  return mPointMatcher->matchStereo(frame);
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

void PointTracker::checkEmptyGrid(const cv::Mat& origin, db::Feature* feature) {
  memset(mGridStatus.data(), 0, sizeof(uint8_t) * mGridStatus.size());
  const auto& uvs = feature->getKeypoints().mUVs;

  const int& rowGridCount = Config::Vio::rowGridCount;
  const int& colGridCount = Config::Vio::colGridCount;
  const int  gridCols     = origin.cols / colGridCount;
  const int  gridRows     = origin.rows / rowGridCount;
  const int  startCol     = (origin.cols % colGridCount) >> 1;
  const int  startRow     = (origin.rows % rowGridCount) >> 1;
  auto       k            = 0u;

  for (const auto& uv : uvs) {
    int col  = (uv.x - startCol) / gridCols;
    int row  = (uv.y - startRow) / gridRows;
    col      = col < 0 ? 0 : col;
    row      = row < 0 ? 0 : row;
    auto idx = colGridCount * row + col;
    if (mGridStatus[idx] != 0x01) {
      mGridStatus[idx] = 0x01;
      k++;
    }
  }
}

cv::Mat PointTracker::createMask(const cv::Mat& origin, db::Feature* feature) {
  cv::Mat mask = cv::Mat(origin.rows, origin.cols, origin.type(), cv::Scalar(255));

  const auto& uvs = feature->getKeypoints().mUVs;

  const int& rowGridCount = Config::Vio::rowGridCount;
  const int& colGridCount = Config::Vio::colGridCount;

  const int gridCols = origin.cols / colGridCount;
  const int gridRows = origin.rows / rowGridCount;
  const int radius   = gridCols > gridRows ? gridRows : gridCols;

  for (const auto& uv : uvs) {
    cv::circle(mask, uv, radius, {0}, -1);
  }
  ////#####################################################################
  //cv::imshow("mask", mask);
  //cv::waitKey(1);
  ////#####################################################################

  return mask;
}

}  //namespace toy