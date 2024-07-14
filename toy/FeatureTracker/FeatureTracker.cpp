#include "ImagePyramid.h"
#include "Frame.h"
#include "PointTracker.h"
#include "LineTracker.h"
#include "FeatureTracker.h"

namespace toy {
FeatureTracker::FeatureTracker(std::string pointTracker, std::string lineTracker)
  : mLineTracker{nullptr} {
  mPointTracker = std::make_unique<PointTracker>(pointTracker);

  if (lineTracker == "none")
    return;

  mLineTracker = std::make_unique<LineTracker>(lineTracker);
}

FeatureTracker::~FeatureTracker() {
  mPointTracker.reset();
  mLineTracker.reset();
}

bool FeatureTracker::process(db::Frame* prevFrame, db::Frame* currentFrame) {
  //YSTODO maybe extract from pyramid...
  mPointTracker->process(prevFrame, currentFrame);

  return true;
}
}  //namespace toy
