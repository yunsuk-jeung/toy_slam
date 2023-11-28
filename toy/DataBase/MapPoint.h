#pragma once
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "macros.h"

namespace toy {
namespace db {
class Frame;
class MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TOY_SMART_PTR(MapPoint);
  TOY_DELETE_COPY_CONSTRUCTORS(MapPoint);
  TOY_DELETE_MOVE_CONSTRUCTORS(MapPoint);

  MapPoint();

protected:

protected:
  int                                               mId;
  std::unordered_map<int, std::weak_ptr<db::Frame>> mFrames;

public:
  static int globalId;
  const int Id() { return mId; }
};

}  //namespace db
}  //namespace toy