#pragma once
#include <tbb/concurrent_set.h>
#include "Frame.h"
#include "Singleton.h"
#include "ToyLogger.h"

namespace toy {
namespace db {

class MemoryPointerPool : public Singleton<MemoryPointerPool> {
public:
  friend class Singleton<MemoryPointerPool>;
  Frame* createFrame(ImagePyramid* in) {
    Frame* out = new Frame(in);
    out->mId   = frameId++;
    mFrames.insert(out);
    return out;
  }

protected:
  uint32_t                    frameId = 0;
  tbb::concurrent_set<Frame*> mFrames;

  MemoryPointerPool(){};
  ~MemoryPointerPool() {
    if (!mFrames.empty()) {
      ToyLogE("You have leaked frames : {} frames are not delete.", mFrames.size());
      for (auto it = mFrames.begin(); it != mFrames.end(); ++it) delete (*it);
    }
  }
};
}  //namespace db
}  //namespace toy
