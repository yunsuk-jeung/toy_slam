#include "ToyLogger.h"
#include "ImagePyramid.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MemoryPointerPool.h"

namespace toy {
namespace db {

uint32_t                    MemoryPointerPool::mFrameId = 0;
tbb::concurrent_set<Frame*> MemoryPointerPool::mFrames;

uint32_t                       MemoryPointerPool::mMapPointId = 0;
tbb::concurrent_set<MapPoint*> MemoryPointerPool::mMapPoints;

void MemoryPointerPool::ready() {}

void MemoryPointerPool::clear() {
  if (!mFrames.empty()) {
    ToyLogE("You have leaked frames : {} frames are not delete.", mFrames.size());
    for (auto it = mFrames.begin(); it != mFrames.end(); ++it) delete (*it);
  }
  if (!mMapPoints.empty()) {
    ToyLogE("You have leaked mapPoints : {} mapPoints are not delete.",
            mMapPoints.size());
    for (auto it = mMapPoints.begin(); it != mMapPoints.end(); ++it) delete (*it);
  }
}

Frame* MemoryPointerPool::createFrame(ImagePyramid* in) {
  Frame* out = new Frame(in);
  out->mId   = mFrameId++;
  mFrames.insert(out);
  return out;
}

MapPoint* MemoryPointerPool::createMapPoint() {
  MapPoint* out = new MapPoint();
  out->mId      = mMapPointId++;
  mMapPoints.insert(out);
  return out;
}

template <>
Frame* MemoryPointerPool::clone<Frame>(Frame* in) {
  Frame* out = new Frame(in);
  mFrames.insert(out);
  return out;
}

template <>
void MemoryPointerPool::release<db::ImagePyramid>(db::ImagePyramid* in) {
  delete in;
}

template <>
void MemoryPointerPool::release<Frame>(Frame* in) {
  if (mFrames.unsafe_erase(in)) { delete in; }
  else { ToyLogE("YOU ARE DELETING NON CREATED MapPoint!!!!!!"); }
}

template <>
void MemoryPointerPool::release<MapPoint>(MapPoint* in) {
  if (mMapPoints.unsafe_erase(in)) { delete in; }
  else { ToyLogE("YOU ARE DELETING NON CREATED FRAME!!!!!!"); }
}

template Frame* MemoryPointerPool::clone<Frame>(Frame* in);

template void MemoryPointerPool::release<db::ImagePyramid>(db::ImagePyramid* in);
template void MemoryPointerPool::release<Frame>(Frame* in);
template void MemoryPointerPool::release<MapPoint>(MapPoint* in);

}  //namespace db
}  //namespace toy