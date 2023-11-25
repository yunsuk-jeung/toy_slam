#pragma once
#include "Map.h"
#include "Frame.h"

namespace toy {
namespace db {
class Frame;
class LocalMap : public Map {
public:
  LocalMap();
  ~LocalMap();

  void   addFramePtr(FramePtr& in);
  Frame* getLatestFrame();

protected:
  std::vector<FramePtr> mFramePtrs;

public:
  std::vector<FramePtr>& getFrameShells() { return mFramePtrs; }
};

}  //namespace db
}  //namespace toy