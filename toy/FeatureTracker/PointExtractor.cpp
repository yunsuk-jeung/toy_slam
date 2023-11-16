#include "Feature.h"
#include "PointExtractor.h"

namespace toy {
PointExtractor::PointExtractor(std::string type) : mType{type} {
  if (mType == "FAST") mFeature2D = cv::FastFeatureDetector::create();
}

PointExtractor::~PointExtractor() {}

void PointExtractor::process(cv::Mat& image, db::Feature* feature) {
  if (mType == "FAST") {
    std::vector<cv::KeyPoint> keyPoints;
    mFeature2D->detect(image, keyPoints);

    setKptsToFeature(keyPoints, feature);
  }
}

void PointExtractor::setKptsToFeature(std::vector<cv::KeyPoint>& kpts,
                                      db::Feature*               feature) {
  auto& keypoints = feature->getKeypoints();
  keypoints.reserve(keypoints.size() + kpts.size());

  auto& ids        = keypoints.mIds;
  auto& levels     = keypoints.mLevels;
  auto& points     = keypoints.mPoints;
  auto& trackCount = keypoints.mTrackCounts;

  for (const auto& kpt : kpts) {
    ids.push_back(mFeatureId++);
    levels.push_back(kpt.octave);
    points.push_back(kpt.pt);
    trackCount.push_back(0);
  }
}

}  //namespace toy