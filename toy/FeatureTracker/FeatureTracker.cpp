#pragma once

#include "Frame.h"
#include "FeatureTracker.h"
#include "PointTracker.h"
// #include "PointTracker.h"

namespace toy {
FeatureTracker::FeatureTracker() {
  mPointTracker = new PointTracker();
}

FeatureTracker::~FeatureTracker() {}

bool FeatureTracker::process(Frame* frame) {

  return true;
}
}  //namespace toy
