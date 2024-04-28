#pragma once
#include <memory>
#include <map>
#include <unordered_map>
#include <forward_list>
#include "Map.h"

namespace toy {
namespace db {
class Frame;
class MapPoint;
class LocalMap : public Map {
public:
  LocalMap();
  ~LocalMap();

  void   reset();
  size_t addFrame(std::shared_ptr<Frame> in);
  void   addMapPoint(std::shared_ptr<MapPoint> in);
  void   getCurrentStates(std::vector<std::shared_ptr<Frame>>&    frames,
                          std::vector<std::shared_ptr<MapPoint>>& trackingMapPoints,
                          std::vector<std::shared_ptr<MapPoint>>& marginedMapPoints);

  void removeFrame(int64_t id);

protected:

protected:
  //in the case of not using sliding window, map is used for frames
  std::map<size_t, std::shared_ptr<Frame>>    mFrames;
  std::map<size_t, std::shared_ptr<MapPoint>> mMapPoints;
  std::map<size_t, std::shared_ptr<MapPoint>> mMapPointCandidates;

public:
  std::map<size_t, std::shared_ptr<Frame>>&    getFrames() { return mFrames; }
  std::map<size_t, std::shared_ptr<MapPoint>>& getMapPoints() { return mMapPoints; }
  std::map<size_t, std::shared_ptr<MapPoint>>& getMapPointCandidiates() {
    return mMapPointCandidates;
  }
};

}  //namespace db
}  //namespace toy