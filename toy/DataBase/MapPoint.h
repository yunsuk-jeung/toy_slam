#pragma once
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "macros.h"
#include "Factor.h"

namespace toy {
namespace db {
class Frame;
class MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  USING_SMART_PTR(MapPoint);
  DELETE_COPY_CONSTRUCTORS(MapPoint);
  DELETE_MOVE_CONSTRUCTORS(MapPoint);

  MapPoint() = delete;
  MapPoint(int id);

  void addFrameFactor(std::shared_ptr<db::Frame> frame, ReprojectionFactor factor);

protected:

public:
  enum class Status {
    INITIALING,
    TRACKING,
    DELETING,
  };

protected:
  using FrameFactorPair = std::pair<std::weak_ptr<db::Frame>, ReprojectionFactor>;

  int                          mId;
  Status                       mStatus;
  std::vector<FrameFactorPair> mFrameFactors;

public:
  const int     Id() const { return mId; }
  const Status& Status() const { return mStatus; }
};

}  //namespace db
}  //namespace toy