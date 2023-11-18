#include "Camera.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "Feature.h"
#include "PointExtractor.h"

namespace toy {
PointExtractor::PointExtractor(std::string type) : mType{type} {
  if (mType == "FAST") mFeature2D = cv::FastFeatureDetector::create();
}

PointExtractor::~PointExtractor() {}

void PointExtractor::process(db::Frame* frame) {
  cv::Mat&     origin  = frame->getImagePyramid(0)->getOrigin();
  db::Feature* feature = frame->getFeature(0);
  Camera*      cam     = frame->getCamera(0);

  if (mType == "FAST") {
    std::vector<cv::KeyPoint> keyPoints;
    mFeature2D->detect(origin, keyPoints);

    convertCVKeyPointsToFeature(cam, keyPoints, feature);
  }
}

cv::Mat PointExtractor::createMask() {
  return cv::Mat();
}

void PointExtractor::convertCVKeyPointsToFeature(Camera*                    cam,
                                                 std::vector<cv::KeyPoint>& kpts,
                                                 db::Feature*               feature) {

  db::Feature newFeat;
  auto&       newKpts = feature->getKeypoints();
  newKpts.reserve(kpts.size());

  auto& ids        = newKpts.mIds;
  auto& levels     = newKpts.mLevels;
  auto& points     = newKpts.mPoints;
  auto& trackCount = newKpts.mTrackCounts;
  auto& undists    = newKpts.mUndsits;

  for (const auto& kpt : kpts) {
    ids.push_back(mFeatureId++);
    levels.push_back(kpt.octave);
    points.push_back(kpt.pt);
    trackCount.push_back(0);
  }

  cam->undistortPoints(points, undists);

  auto& keypoints = feature->getKeypoints();
  keypoints.push_back(newKpts);
}

}  //namespace toy