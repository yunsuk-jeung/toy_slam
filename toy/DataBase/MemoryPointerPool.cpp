#include "ToyLogger.h"
#include "MemoryPointerPool.h"

namespace toy {
namespace db {

uint32_t                    MemoryPointerPool::mFrameId = 0;
tbb::concurrent_set<Frame*> MemoryPointerPool::mFrames;

void MemoryPointerPool::ready() {}

void MemoryPointerPool::clear() {
  if (!mFrames.empty()) {
    ToyLogE("You have leaked frames : {} frames are not delete.", mFrames.size());
    for (auto it = mFrames.begin(); it != mFrames.end(); ++it) delete (*it);
  }
}

Frame* MemoryPointerPool::createFramePtr(ImagePyramid* in) {
  Frame* out = new Frame(in);
  out->mId   = mFrameId++;
  mFrames.insert(out);
  return out;
}

template <>
Frame* MemoryPointerPool::clone<Frame>(Frame* in) {
  Frame* out = new Frame(in);
  mFrames.insert(out);
  return out;
}

template <>
void MemoryPointerPool::release<Frame>(Frame* in) {
  if (mFrames.unsafe_erase(in)) { delete in; }
  else { ToyLogE("YOU ARE DELETING NON CREATED FRAME!!!!!!"); }
}

template Frame* MemoryPointerPool::clone<Frame>(Frame* in);

template void MemoryPointerPool::release<Frame>(Frame* in);

}  //namespace db
}  //namespace toy