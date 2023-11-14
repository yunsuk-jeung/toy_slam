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
                               std::string lineMatcher) {

  mPointExtractorUptr = std::make_unique<PointExtractor>(pointExtractor);
  mPointMatcherUptr   = std::make_unique<PointMatcher>(pointMatcher);

  if (lineExtractor == "none") return;

  mLineExtractorUptr = std::make_unique<LineExtractor>(lineExtractor);
  mLineMatcherUptr   = std::make_unique<LineMatcher>(lineMatcher);
}

FeatureTracker::~FeatureTracker() {}

bool FeatureTracker::process(Frame* frame) {

  return true;
}

}  //namespace toy
