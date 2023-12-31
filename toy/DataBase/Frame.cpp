#include "Frame.h"
#include "Camera.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "ToyLogger.h"

namespace toy {
namespace db {
int Frame::globalId = 0;
Frame::Frame(std::shared_ptr<ImagePyramidSet> set)
  : mId{globalId++}
  , mImagePyramids{std::move(set->mImagePyramid0), std::move(set->mImagePyramid1)}
  , mCameras{nullptr, nullptr}
  , mFeatures{Feature::Uni(new Feature()), Feature::Uni(new Feature())}
  , mLbcs{Eigen::Vector6d(), Eigen::Vector6d()} {}

Frame::Frame(Frame* src) {
  this->mId = src->mId;

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
}

Frame::~Frame() {
  for (ImagePyramid::Uni& ptr : mImagePyramids) { ptr.reset(); }
  for (Camera::Uni& ptr : mCameras) { ptr.reset(); }
  for (Feature::Uni& ptr : mFeatures) { ptr.reset(); }
}

Frame::Ptr Frame::clonePtr() {
  Frame::Ptr out(new Frame(this));
  return out;
}

void Frame::setCameras(Camera* cam0, Camera* cam1) {
  mCameras[0] = std::unique_ptr<Camera>(cam0);
  mCameras[1] = std::unique_ptr<Camera>(cam1);
}

void Frame::setLbc(float* pfbc0, float* pfbc1) {
  Eigen::Matrix4f Mbc0(pfbc0);
  Eigen::Matrix4f Mbc1(pfbc1);

  Eigen::Matrix3d Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d Rbc1 = Mbc1.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc1 = Mbc1.block<3, 1>(0, 3).cast<double>();

  Eigen::Quaterniond Qbc0(Rbc0);
  Eigen::Quaterniond Qbc1(Rbc1);

  Sophus::SO3d SObc0 = Sophus::SO3d(Qbc0);
  mLbcs[0].head(3)   = SObc0.log();
  mLbcs[0].tail(3)   = Tbc0;

  Sophus::SO3d SObc1 = Sophus::SO3d(Qbc1);
  mLbcs[1].head(3)   = SObc1.log();
  mLbcs[1].tail(3)   = Tbc1;
}

}  //namespace db
}  //namespace toy
