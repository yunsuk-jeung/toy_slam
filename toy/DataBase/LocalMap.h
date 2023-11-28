#pragma once
#include <memory>
#include <map>
#include <unordered_map>
#include "Map.h"

namespace toy {
namespace db {
class Frame;
class MapPoint;
class LocalMap : public Map {
public:
  LocalMap();
  ~LocalMap();

  void                   reset();
  void                   addFrame(std::shared_ptr<Frame> in);
  std::shared_ptr<Frame> getLatestFrame();

protected:
  void createMapPoints();

protected:
  //in the case of not using sliding window, map is used for frames
  std::map<int, std::shared_ptr<Frame>>              mFrames;
  std::unordered_map<int, std::shared_ptr<MapPoint>> mMapPoints;

public:
  std::map<int, std::shared_ptr<Frame>>& getFrames() { return mFrames; }
};

}  //namespace db
}  //namespace toy