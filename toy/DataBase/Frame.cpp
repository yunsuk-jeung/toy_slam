#include <opencv2/opencv.hpp>
#include "ToyLogger.h"
#include "Camera.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "MapPoint.h"
#include "FrameState.h"
#include "Frame.h"

namespace toy {
namespace db {

//Frame::Frame(FrameState* frameState)
//  : mTwb(frameState->getTwb()) {
//  this->mImagePyramid = src->mImagePyramid;
//  Camera* cam0        = src->mCamera->clone();
//  this->mCamera       = std::unique_ptr<Camera>(cam0);
//
//  auto* feature  = src->mFeature->clone();
//  this->mFeature = std::unique_ptr<Feature>(feature);
//
//  this->mTbc = src->mTbc;
//  this->mTwb = src->mTwb;
//}

//Frame::~Frame() {
//  mCamera.reset();
//  mFeature.reset();
//}

Frame::Frame(FrameState*                       frameState,
             size_t                            cameraId,
             std::shared_ptr<db::ImagePyramid> imagePyramid,
             std::unique_ptr<Camera>           camera)
  : mFrameState{frameState}
  , mTwb(frameState->getTwb())
  , mCameraId{cameraId}
  , mImagePyramid{imagePyramid} {
  mCamera  = std::move(camera);
  mFeature = std::make_unique<Feature>();
}

Frame::Ptr Frame::clone(FrameState* fs) {
  return std::make_shared<Frame>(this);
}

void Frame::setTbc(float* pfbc) {
  Eigen::Matrix4f    Mbc0(pfbc);
  Eigen::Matrix3d    Rbc0 = Mbc0.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d    Tbc0 = Mbc0.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond Qbc0(Rbc0);

  mTbc = Sophus::SE3d(Qbc0, Tbc0);
}

void Frame::addMapPointFactor(std::shared_ptr<db::MapPoint> mp,
                              ReprojectionFactor            factor) {
  mMapPointFactorMap.insert({mp, factor});
}

void Frame::drawReprojectionView(std::string imshowName, bool half) {
  auto img = mImagePyramid->getOrigin().clone();
  cv::cvtColor(img, img, CV_GRAY2BGR);

  auto Tcw = getTwc().inverse();

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
      color = {0, 0, 0};
      break;
      //continue;
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

    undist = factor.undist();

    auto meaUV = mCamera->project(undist);
    cv::circle(img, meaUV, 6, {255, 0, 0}, -1);

    Eigen::Vector3d Xcx  = Tcw * mp->getPwx();
    Eigen::Vector3d nXcx = Xcx / Xcx.z();

    auto proj = mCamera->project(nXcx);

    cv::circle(img, proj, 3, color, -1);
    if (undist.z() > 0 && mp->status() > db::MapPoint::Status::NONE) {
      cv::line(img, meaUV, proj, color);
    }

    cv::putText(img,
                std::to_string(mp->id()),
                meaUV + cv::Point2d(5, 5),
                cv::FONT_HERSHEY_PLAIN,
                1,
                {255, 0, 0});
  }

  if (mFrameState->isKeyFrame()) {
    cv::putText(img,
                std::to_string(mFrameState->id()),
                cv::Point2f(50, 50),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(0, 0, 255),
                3);
  }
  else {
    cv::putText(img,
                std::to_string(mFrameState->id()),
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
