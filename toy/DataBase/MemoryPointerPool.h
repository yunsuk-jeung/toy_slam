#pragma once
#include <tbb/concurrent_set.h>

namespace toy {
namespace db {
class ImagePyramid;
class Frame;
class MapPoint;
class MemoryPointerPool {
public:
  static void ready();
  static void clear();

  static Frame*    createFrame(ImagePyramid* in);
  static MapPoint* createMapPoint();

  template <typename T>
  static T* clone(T* in);

  template <typename T>
  static void release(T* in);

protected:
  static uint32_t                    mFrameId;
  static tbb::concurrent_set<Frame*> mFrames;

  static uint32_t                       mMapPointId;
  static tbb::concurrent_set<MapPoint*> mMapPoints;
};
}  //namespace db
}  //namespace toy
