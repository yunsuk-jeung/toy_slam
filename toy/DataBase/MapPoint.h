#pragma once
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "macros.h"
#include "Factor.h"
#include "CustomTypes.h"

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
  MapPoint(int64_t id);

  void addFrameFactor(std::shared_ptr<db::Frame> frame, ReprojectionFactor factor);

  void backup();
  void restore();
  void update(const Eigen::Vector3d& delta);
  void update(const double& delta);

  uint8_t eraseFrame(std::shared_ptr<db::Frame> frame);

protected:

public:
  enum class Status {
    DELETING = -2,
    NONE     = 0,
    MARGINED = 1,
    TRACKING = 2,
  };

protected:
  using FrameFactorMap = std::map<FrameCamId, ReprojectionFactor>;

  int64_t                    mId;
  Status                     mStatus;
  std::shared_ptr<db::Frame> mHostFrame;
  Eigen::Vector2d            mUndist;
  double                     mInvDepth;
  Eigen::Vector2d            mBackupUndist;
  double                     mBackupInvD;
  bool                       mFixed;
  Eigen::Vector3d            mMarginedPwx;
  FrameFactorMap             mFrameFactorMap;

public:
  const int64_t id() const { return mId; }
  const Status& status() const { return mStatus; }
  void          setState(Status status) { mStatus = status; }

  FrameFactorMap& frameFactorMap() { return mFrameFactorMap; }

  void                       setHost(std::shared_ptr<db::Frame> host);
  std::shared_ptr<db::Frame> hostFrame() { return mHostFrame; }

  const Eigen::Vector2d& undist() const { return mUndist; }
  void                   setUndist(const Eigen::Vector2d& undist) { mUndist = undist; }
  Eigen::Vector2d&       getUndist() { return mUndist; }

  const double    invDepth() const { return mInvDepth; }
  void            setInvDepth(double& invD) { mInvDepth = invD; }
  double&         getInvDepth() { return mInvDepth; }
  Eigen::Vector3d getPwx();

  const bool fixed() const { return mFixed; }
  void       setFixed(bool fixed) { mFixed = fixed; }

  static constexpr size_t UNDIST_SIZE    = 2;
  static constexpr size_t INVDEPTH_SIZE  = 1;
  static constexpr size_t PARAMETER_SIZE = 3;
};

}  //namespace db
}  //namespace toy