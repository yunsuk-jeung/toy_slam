#pragma once
#include "Map.h"

namespace toy {
namespace db {
class Frame;
class LocalMap : public Map {
public:
  LocalMap();
  ~LocalMap();

  void   addFrame(Frame* in);
  Frame* getLatestFrame();

protected:
  std::vector<Frame*> mFrames;

public:
  std::vector<Frame*>& getFrames() { return mFrames; }
};

}  //namespace db
}  //namespace toy