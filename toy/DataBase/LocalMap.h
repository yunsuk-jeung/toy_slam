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

  void reset();
  bool addFrame(std::shared_ptr<Frame> in);
  void getCurrentStates(std::vector<std::shared_ptr<Frame>>&    frames,
                        std::vector<std::shared_ptr<MapPoint>>& mapPoints);

protected:

protected:
  //in the case of not using sliding window, map is used for frames
  std::map<int, std::shared_ptr<Frame>>    mFrames;
  std::map<int, std::shared_ptr<MapPoint>> mMapPoints;

public:
  std::map<int, std::shared_ptr<Frame>>&    getFrames() { return mFrames; }
  std::map<int, std::shared_ptr<MapPoint>>& getMapPoints() { return mMapPoints; }
};

}  //namespace db
}  //namespace toy