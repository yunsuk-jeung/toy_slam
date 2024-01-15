#include <opencv2/opencv.hpp>
#include "ToyLogger.h"
#include "Camera.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "MapPoint.h"
#include "Frame.h"

namespace toy {
namespace db {
size_t Frame::globalId = 0;
Frame::Frame(std::shared_ptr<ImagePyramidSet> set)
  : mId{globalId++}
  , mIsKeyFrame{false}
  , mImagePyramids{std::move(set->mImagePyramid0), std::move(set->mImagePyramid1)}
  , mCameras{nullptr, nullptr}
  , mFeatures{std::make_unique<Feature>(), std::make_unique<Feature>()}
  , mFixed{false}
  , mLinearized{false} {}

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
  this->mLinearized  = src->mLinearized;
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

void Frame::drawReprojectionView(int idx, std::string imshowName, bool half) {
  auto img = mImagePyramids[idx]->getOrigin().clone();
  cv::cvtColor(img, img, CV_GRAY2BGR);

  auto Tcw = getTwc(idx).inverse();

  for (auto& [mpw, factor] : mMapPointFactorMap) {
    auto mp = mpw.lock();
    if (!mp) {
      continue;
    }

    cv::Scalar color;
    switch (mp->status()) {
    case db::MapPoint::Status::DELETING: {
      continue;
    }
    case db::MapPoint::Status::NONE: {
      //color = {0, 0, 0};
      continue;
    }
    case db::MapPoint::Status::MARGINED: {
      color = {255, 255, 0};
      break;
    }
    case db::MapPoint::Status::INITIALING: {
      color = {0, 255, 0};
      break;
    }
    case db::MapPoint::Status::TRACKING: {
      color = {0, 0, 255};
      break;
    }
    }

    Eigen::Vector3d undist;

    if (idx == 0) {
      undist = factor.undist0();
    }
    else {
      undist = factor.undist1();
    }

    auto meaUV = mCameras[idx]->project(undist);
    cv::circle(img, meaUV, 6, {255, 0, 0}, -1);

    Eigen::Vector3d Xcx  = Tcw * mp->getPwx();
    Eigen::Vector3d nXcx = Xcx / Xcx.z();

    auto proj = mCameras[idx]->project(nXcx);

    cv::circle(img, proj, 3, color, -1);
    if (undist.z() > 0) {
      cv::line(img, meaUV, proj, color);
    }

    cv::putText(img,
                std::to_string(mp->id()),
                meaUV + cv::Point2d(5, 5),
                cv::FONT_HERSHEY_PLAIN,
                1,
                {255, 0, 0});
  }

  if (this->isKeyFrame()) {
    cv::putText(img,
                std::to_string(this->id()),
                cv::Point2f(50, 50),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(0, 0, 255),
                3);
  }
  else {
    cv::putText(img,
                std::to_string(this->id()),
                cv::Point2f(50, 50),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(0, 0, 0),
                3);
  }
  if (half) {
    auto halfSize = cv::Size(img.cols / 2, img.rows / 2);
    cv::resize(img, img, halfSize);
  }
  cv::imshow(imshowName, img);
  cv::waitKey(1);
}

}  //namespace db
}  //namespace toy
