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
  size_t addFrameState(std::shared_ptr<FrameState> in);
  void   addMapPoint(std::shared_ptr<MapPoint> in);
  void   getCurrentStates(std::vector<std::shared_ptr<FrameState>>& frames,
                          std::vector<std::shared_ptr<MapPoint>>& trackingMapPoints,
                          std::vector<std::shared_ptr<MapPoint>>& marginedMapPoints);

  void removeFrame(int id);

protected:

protected:
  //in the case of not using sliding window, map is used for frames
  std::map<size_t, std::shared_ptr<FrameState>> mFrameStates;
  std::map<size_t, std::shared_ptr<MapPoint>>   mMapPoints;
  std::map<size_t, std::shared_ptr<MapPoint>>   mMapPointCandidates;

public:
  std::map<size_t, std::shared_ptr<FrameState>>& getFrames() { return mFrameStates; }
  std::map<size_t, std::shared_ptr<MapPoint>>&   getMapPoints() { return mMapPoints; }
  std::map<size_t, std::shared_ptr<MapPoint>>&   getMapPointCandidiates() {
    return mMapPointCandidates;
  }
};

}  //namespace db
}  //namespace toy