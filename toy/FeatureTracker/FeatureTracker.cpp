#pragma once
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

bool FeatureTracker::process(Frame* frame) {

  return true;
}

}  //namespace toy
