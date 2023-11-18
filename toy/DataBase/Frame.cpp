#include "ToyLogger.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "Frame.h"

namespace toy {
namespace db {

Frame::Frame(ImagePyramid* imagePyramid)
    : mId{-1}
    , mImagePyramid0{&imagePyramid[0]}
    , mImagePyramid1{&imagePyramid[1]}
    , mFeature0{new Feature()}
    , mFeature1{new Feature()} {}

Frame::~Frame() {
  delete[] mImagePyramid0;
  mImagePyramid0 = nullptr;
  mImagePyramid1 = nullptr;

  delete mCam0;
  mCam0 = nullptr;

  delete mCam1;
  mCam1 = nullptr;

  delete mFeature0;
  mFeature0 = nullptr;

  delete mFeature1;
  mFeature1 = nullptr;
}

void Frame::setPbc(float* pfbc0, float* pfbc1) {
  Eigen::Matrix4f Mbc0(pfbc0);
  Eigen::Matrix4f Mbc1(pfbc1);

  Eigen::Matrix3d Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d Rbc1 = Mbc1.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d Tbc1 = Mbc1.block<3, 1>(0, 3).cast<double>();

  Eigen::Quaterniond Qbc0(Rbc0);
  Eigen::Quaterniond Qbc1(Rbc1);
  Pbc0 = Sophus::SE3d(Qbc0, Tbc0);
  Pbc1 = Sophus::SE3d(Qbc1, Tbc1);

  //ToyLogD("SE3 log test : {}", LogUtil::SE3String(Pbc0));
  //ToyLogD("se3 log test : {}", LogUtil::se3String(Pbc1));
}

}  //namespace db
}  //namespace toy
