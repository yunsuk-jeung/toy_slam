#include "MapPoint.h"
namespace toy {
namespace db {

MapPoint::MapPoint(int id)
  : mStatus{Status::INITIALING} {
  mId = id;
  mFrameFactors.reserve(50);
}

void MapPoint::addFrameFactor(std::shared_ptr<db::Frame> frame,
                              ReprojectionFactor         factor) {
  mFrameFactors.push_back({frame, factor});
}

}  //namespace db
}  //namespace toy
