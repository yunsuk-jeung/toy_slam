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

  if (lineTracker == "none") return;

  mLineTracker = new LineTracker(lineTracker);
}

FeatureTracker::~FeatureTracker() {
  delete mPointTracker;
  mPointTracker = nullptr;

  delete mLineTracker;
  mLineTracker;
}

bool FeatureTracker::process(db::Frame* frame) {

  //todo maybe extract from pyramid...
  mPointTracker->process(frame);

  return true;
}

}  //namespace toy
