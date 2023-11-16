#pragma once
#include "ImagePyramid.h"
#include "Frame.h"
#include "PointExtractor.h"
#include "PointMatcher.h"
#include "LineExtractor.h"
#include "LineMatcher.h"
#include "FeatureTracker.h"

namespace toy {
FeatureTracker::FeatureTracker(std::string pointExtractor,
                               std::string pointMatcher,
                               std::string lineExtractor,
                               std::string lineMatcher)
    : mLineExtractor{nullptr}, mLineMatcher{nullptr} {

  mPointExtractor = new PointExtractor(pointExtractor);
  mPointMatcher   = new PointMatcher(pointMatcher);

  if (lineExtractor == "none") return;

  mLineExtractor = new LineExtractor(lineExtractor);
  mLineMatcher   = new LineMatcher(lineMatcher);
}

FeatureTracker::~FeatureTracker() {
  delete mPointExtractor;
  mPointExtractor = nullptr;

  delete mPointMatcher;
  mPointMatcher;

  delete mLineExtractor;
  mLineExtractor = nullptr;

  delete mLineMatcher;
  mLineMatcher = nullptr;
}

bool FeatureTracker::process(db::Frame* frame) {
  cv::Mat&     origin0  = frame->getImagePyramid0()->getOrigin();
  db::Feature* feature0 = frame->getFeature0();

  //todo maybe extract from pyramid...
  mPointExtractor->process(origin0, feature0);

  if (frame->getImagePyramid1()->type() == 1) {

    db::Feature* feature1 = frame->getFeature1();
    mPointMatcher->process(frame->getImagePyramid0(),
                           feature0,
                           frame->getImagePyramid1(),
                           feature1);
  }

  return true;
}

}  //namespace toy
