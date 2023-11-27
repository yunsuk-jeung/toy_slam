#pragma once
#include <unordered_map>
#include "Frame.h"

namespace toy {
namespace db {

class MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class MemoryPointerPool;

protected:
  MapPoint() = default;

protected:
  int                                   mId;
  std::unordered_map<int, db::FramePtr> mFramePtrs;

public:
  const int Id() { return mId; }
};

class MapPointPtr : public Pointer<MapPoint> {
public:
  MapPointPtr() = default;

  explicit MapPointPtr(MapPoint* mp)
    : Pointer<MapPoint>(mp) {}

  ~MapPointPtr() = default;

  void release() override;
};

}  //namespace db
}  //namespace toy