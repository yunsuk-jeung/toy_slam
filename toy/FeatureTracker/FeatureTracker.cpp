#pragma once
#include "ImagePyramid.h"
#include "Frame.h"
#include "PointTracker.h"
#include "LineTracker.h"
#include "FeatureTracker.h"

namespace toy {
FeatureTracker::FeatureTracker(std::string pointTracker, std::string lineTracker)
  : mLineTracker{nullptr} {
  mPointTracker = new PointTracker(pointTracker);

  if (lineTracker == "none")
    return;

  mLineTracker = new LineTracker(lineTracker);
}

FeatureTracker::~FeatureTracker() {
  delete mPointTracker;
  mPointTracker = nullptr;

  delete mLineTracker;
  mLineTracker;
}

bool FeatureTracker::process(db::Frame* prevFrame, db::Frame* currentFrame) {
  //YSTODO maybe extract from pyramid...
  mPointTracker->process(prevFrame, currentFrame);

  return true;
}
}  //namespace toy
