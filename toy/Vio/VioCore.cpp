#include "ImagePyramid.h"
#include "FrameTracker.h"
#include "LocalTracker.h"

#include "VioCore.h"

namespace toy {
VioCore::VioCore()
  : mFrameTracker{nullptr}
  , mLocalTracker{nullptr} {}

VioCore::~VioCore() {
  delete mFrameTracker;
  mFrameTracker = nullptr;
}

void VioCore::insert(db::ImagePyramid* imagePyramid) {
  mFrameTracker->insert(imagePyramid);
}

void VioCore::prepare() {
  mFrameTracker = new FrameTracker();
  mLocalTracker = new LocalTracker();

  mFrameTracker->registerOutQueue(&(mLocalTracker->getInQueue()));

  mFrameTracker->prepare();
  mLocalTracker->prepare();
}

void VioCore::processSync() {
  mFrameTracker->process();
  mLocalTracker->process();
}

}  //namespace toy