#include "ToyLogger.h"
#include "LocalTracker.h"

namespace toy {
LocalTracker::LocalTracker() {}

LocalTracker::~LocalTracker() {}

void LocalTracker::prepare() {}

void LocalTracker::process() {
  ToyLogD("localTracker in queue size : {}", mInQueue.unsafe_size());
}

}  //namespace toy