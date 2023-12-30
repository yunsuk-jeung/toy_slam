#include "ToyLogger.h"
#include "Camera.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "MapPoint.h"
#include "Frame.h"

namespace toy {
namespace db {
int Frame::globalId = 0;
Frame::Frame(std::shared_ptr<ImagePyramidSet> set)
  : mId{globalId++}
  , mIsKeyFrame{false}
  , mImagePyramids{std::move(set->mImagePyramid0), std::move(set->mImagePyramid1)}
  , mCameras{nullptr, nullptr}
  , mFeatures{std::make_unique<Feature>(), std::make_unique<Feature>()}
  , mFixed{false} {}

Frame::Frame(Frame* src) {
  this->mId         = src->mId;
  this->mIsKeyFrame = src->mIsKeyFrame;

  db::ImagePyramid* pyramid0 = src->mImagePyramids[0]->clone();
  db::ImagePyramid* pyramid1 = src->mImagePyramids[1]->clone();
  this->mImagePyramids[0]    = std::unique_ptr<db::ImagePyramid>(pyramid0);
  this->mImagePyramids[1]    = std::unique_ptr<db::ImagePyramid>(pyramid1);

  Camera* cam0      = src->mCameras[0]->clone();
  Camera* cam1      = src->mCameras[1]->clone();
  this->mCameras[0] = std::unique_ptr<Camera>(cam0);
  this->mCameras[1] = std::unique_ptr<Camera>(cam1);

  auto* feature0     = src->mFeatures[0]->clone();
  auto* feature1     = src->mFeatures[1]->clone();
  this->mFeatures[0] = std::unique_ptr<Feature>(feature0);
  this->mFeatures[1] = std::unique_ptr<Feature>(feature1);

  this->mTbcs        = src->mTbcs;
  this->mTwb         = src->mTwb;
  this->mBackupTwb   = src->mBackupTwb;
  this->mDelta       = src->mDelta;
  this->mBackupDelta = src->mBackupDelta;
  this->mFixed       = src->mFixed;
}

Frame::~Frame() {
  for (ImagePyramid::Uni& ptr : mImagePyramids) {
    ptr.reset();
  }
  for (Camera::Uni& ptr : mCameras) {
    ptr.reset();
  }
  for (Feature::Uni& ptr : mFeatures) {
    ptr.reset();
  }
}

Frame::Ptr Frame::clonePtr() {
  return std::make_shared<Frame>(this);
}

void Frame::setCameras(Camera* cam0, Camera* cam1) {
  mCameras[0] = std::unique_ptr<Camera>(cam0);
  mCameras[1] = std::unique_ptr<Camera>(cam1);
}

//void Frame::setLbc(float* pfbc0, float* pfbc1) {
//  Eigen::Matrix4f Mbc0(pfbc0);
//  Eigen::Matrix4f Mbc1(pfbc1);

//Eigen::Matrix3d Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
//Eigen::Vector3d Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();

//Eigen::Matrix3d Rbc1 = Mbc1.block<3, 3>(0, 0).cast<double>();
//Eigen::Vector3d Tbc1 = Mbc1.block<3, 1>(0, 3).cast<double>();

//Eigen::Quaterniond Qbc0(Rbc0);
//Eigen::Quaterniond Qbc1(Rbc1);

//Sophus::SO3d SObc0 = Sophus::SO3d(Qbc0);
//mLbcs[0].head(3)   = SObc0.log();
//mLbcs[0].tail(3)   = Tbc0;

//Sophus::SO3d SObc1 = Sophus::SO3d(Qbc1);
//mLbcs[1].head(3)   = SObc1.log();
//mLbcs[1].tail(3)   = Tbc1;
//}

void Frame::setSbc(float* pfbc0, float* pfbc1) {
  Eigen::Matrix4f Mbc0(pfbc0);
  Eigen::Matrix4f Mbc1(pfbc1);

  Eigen::Matrix3d Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d Rbc1 = Mbc1.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc1 = Mbc1.block<3, 1>(0, 3).cast<double>();

  Eigen::Quaterniond Qbc0(Rbc0);
  Eigen::Quaterniond Qbc1(Rbc1);

  mTbcs[0] = Sophus::SE3d(Qbc0, Tbc0);
  mTbcs[1] = Sophus::SE3d(Qbc1, Tbc1);
}

void Frame::addMapPointFactor(std::shared_ptr<db::MapPoint> mp,
                              ReprojectionFactor            factor) {
  mMapPointFactorMap.insert({mp, factor});
}

void Frame::resetDelta() {
  mDelta.setZero();
}

void Frame::backup() {
  mBackupTwb   = mTwb;
  mBackupDelta = mDelta;
}

void Frame::restore() {
  mDelta = mBackupDelta;
  mTwb   = mBackupTwb;
}

void Frame::update(const Eigen::Vector6d& delta) {
  auto& Pwb = mTwb.translation();
  Pwb += delta.head(3);

  auto& so3wb = mTwb.so3();
  so3wb *= Sophus::SO3d::exp(delta.tail(3));

  mDelta += delta;
}

}  //namespace db
}  //namespace toy
