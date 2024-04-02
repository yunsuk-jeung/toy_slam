#include <opencv2/opencv.hpp>
#include "ToyAssert.h"
#include "Camera.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "MapPoint.h"
#include "Frame.h"
#include "FrameState.h"

namespace toy {
namespace db {
size_t FrameState::globalId = 0;
FrameState::FrameState(std::shared_ptr<ImagePyramidSet> set,
                       std::vector<CameraInfo>&         cameraInfos)
  : mId{globalId++}
  , mIsKeyFrame{false}
  , mFixed{false}
  , mLinearized{false} {
  TOY_ASSERT(set->images_.size() == cameraInfos.size());
  set->images_;
}

FrameState::~FrameState() {
  mFrames.clear();
}

FrameState::Ptr FrameState::clone() {
  auto out = std::make_shared<FrameState>();

  out->mId          = this->mId;
  out->mIsKeyFrame  = this->mIsKeyFrame;
  out->mTwb         = this->mTwb;
  out->mBackupTwb   = this->mBackupTwb;
  out->mDelta       = this->mDelta;
  out->mBackupDelta = this->mBackupDelta;
  out->mFixed       = this->mFixed;
  out->mLinearized  = this->mLinearized;

  auto frameSize = this->mFrames.size();
  out->mFrames.reserve(frameSize);
  for (auto src : this->mFrames) {
    auto dst = src->clone(out.get());
    this->mFrames.push_back(dst);
  }

  return out;
}

//void FrameState::setSbc(float* pfbc0, float* pfbc1) {
//  Eigen::Matrix4f Mbc0(pfbc0);
//  Eigen::Matrix4f Mbc1(pfbc1);
//
//  Eigen::Matrix3d Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
//  Eigen::Vector3d Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();
//
//  Eigen::Matrix3d Rbc1 = Mbc1.block<3, 3>(0, 0).cast<double>();
//  Eigen::Vector3d Tbc1 = Mbc1.block<3, 1>(0, 3).cast<double>();
//
//  Eigen::Quaterniond Qbc0(Rbc0);
//  Eigen::Quaterniond Qbc1(Rbc1);
//
//  mTbcs[0] = Sophus::SE3d(Qbc0, Tbc0);
//  mTbcs[1] = Sophus::SE3d(Qbc1, Tbc1);
//}

//void FrameState::addMapPointFactor(std::shared_ptr<db::MapPoint> mp,
//                                   ReprojectionFactor            factor) {
//  mMapPointFactorMap.insert({mp, factor});
//}

void FrameState::resetDelta() {
  mDelta.setZero();
}

void FrameState::backup() {
  mBackupTwb   = mTwb;
  mBackupDelta = mDelta;
}

void FrameState::restore() {
  mDelta = mBackupDelta;
  mTwb   = mBackupTwb;
}

void FrameState::update(const Eigen::Vector6d& delta) {
  auto& Pwb = mTwb.translation();
  Pwb += delta.head(3);

  auto& so3wb = mTwb.so3();
  so3wb *= Sophus::SO3d::exp(delta.tail(3));

  mDelta += delta;
}
}  //namespace db
}  //namespace toy
