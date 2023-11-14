#include "Logger.h"
#include "LocalMap.h"
namespace toy {
namespace db {
LocalMap::LocalMap() {}

LocalMap::~LocalMap() {
  if (!mFrames.empty()) {
    LOGE("You have leaked frame mFrames : {} frames are not delete.", mFrames.size());
    for (auto it = mFrames.begin(); it != mFrames.end(); ++it) delete (*it);
  }
}

Frame* LocalMap::createNewFrame(ImagePyramid* in) {
  Frame* out = new Frame(in);
  mFrames.insert(out);
  return out;
}

}  //namespace db
}  //namespace toy