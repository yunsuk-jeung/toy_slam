#pragma once
#include <array>
#include <memory>
#include <map>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "usings.h"
#include "macros.h"
#include "Factor.h"

namespace toy {
class Camera;
namespace db {
class ImagePyramid;
class ImagePyramidSet;
class LocalMap;
class Feature;
class MapPoint;
class Frame;
class FrameState {
public:
  USING_SMART_PTR(FrameState);
  DELETE_COPY_CONSTRUCTORS(FrameState);
  static constexpr size_t PARAMETER_SIZE = 6;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrameState(std::shared_ptr<ImagePyramidSet> set, std::vector<CameraInfo>& cameraInfos);
  ~FrameState();

  FrameState::Ptr clone();

  void resetDelta();
  void backup();
  void restore();
  void update(const Eigen::Vector6d& delta);

  Eigen::Vector<double, PARAMETER_SIZE> toParameter() const {
    Eigen::Vector<double, PARAMETER_SIZE> out;
    out.head(3) = this->Twb().translation();
    out.tail(3) = this->Twb().so3().log();
    return out;
  }

protected:
  FrameState() = default;

protected:
  static size_t                       globalId;
  size_t                              mId;
  bool                                mIsKeyFrame;
  std::vector<std::shared_ptr<Frame>> mFrames;

  //S : se3
  std::array<Sophus::SE3d, 2> Tbcs;

  bool mFixed;
  bool mLinearized;

  Sophus::SE3d mTwb;
  Sophus::SE3d mBackupTwb;

  Eigen::Vector6d mDelta;
  Eigen::Vector6d mBackupDelta;

public:
  const size_t        id() const { return mId; }
  void                setKeyFrame() { mIsKeyFrame = true; }
  const bool          isKeyFrame() const { return mIsKeyFrame; }
  auto&               getFrames() { return mFrames; }
  void                setTwb(const Sophus::SE3d& Twb) { mTwb = Twb; }
  const Sophus::SE3d& Twb() const { return mTwb; }
  Sophus::SE3d&       getTwb() { return mTwb; }
  const bool          fixed() const { return mFixed; }
  void                setFixed(bool fixed) { mFixed = fixed; }
  void                setLinearized(bool linearized) { mLinearized = linearized; }
  bool                isLinearized() { return mLinearized; }
  Eigen::Vector6d     getDelta() { return mDelta; };
};

}  //namespace db
}  //namespace toy