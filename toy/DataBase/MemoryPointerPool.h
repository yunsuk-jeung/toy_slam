#pragma once
#include <tbb/concurrent_set.h>

namespace toy {
namespace db {
class Frame;
class MemoryPointerPool {
public:
  static void ready();
  static void clear();

  static Frame* createFrame(ImagePyramid* in);

  template <typename T>
  static T* clone(T* in);

  template <typename T>
  static void release(T* in);

protected:
  static uint32_t                    mFrameId;
  static tbb::concurrent_set<Frame*> mFrames;
};
}  //namespace db
}  //namespace toy
