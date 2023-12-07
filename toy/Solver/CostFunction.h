#pragma once
#include "macros.h"
#include "usings.h"
#include "Frame.h"
#include "MapPoint.h"
#include "Factor.h"
#include "EigenUtil.h"

namespace toy {
class MEstimator {
public:
  USING_SMART_PTR(MEstimator);
  MEstimator() = delete;
  MEstimator(double c = 1.0)
    : mC{c} {}

protected:
  double mC;  //constant
};

class Huber : public MEstimator {
public:
  USING_SMART_PTR(Huber);
  Huber(double c = 1.0)
    : MEstimator(c) {}
};

class PoseOnlyReprojectionCost {
public:
  PoseOnlyReprojectionCost(Eigen::Vector6d*  Lwc,
                           db::MapPoint::Ptr mp,
                           Eigen::Vector2d   undist,
                           MEstimator::Ptr   ME)
    : mLwc{Lwc}
    , mMapPoint{mp}
    , mJ{Eigen::Matrix23d::Zero()}
    , mR{Eigen::Vector2d::Zero()}
    , mZ{undist}
    , mME{ME} {}

  void linearlize() {
    Eigen::Vector3d Pwx = mMapPoint->getPwx();

    Sophus::SE3d Scw = Sophus::SE3d::exp(*mLwc).inverse();

    Eigen::Vector3d Pcx = Scw * Pwx;
    double          x, y, z;

    x = Pcx.x();
    y = Pcx.y();
    z = Pcx.z();

    double iz   = 1.0 / z;
    double izSq = iz * iz;

    Eigen::Matrix23d reduce;
    reduce << iz, 0.0, -x * izSq, 0.0, iz, -y * izSq;

    Eigen::Vector3d undist = Pcx / Pcx.z();
    Eigen::Vector2d res = mZ - undist;

    //Eigen::Matrix3d Rcw    = Scw.so3().matrix();
    //Eigen::Matrix3d p_skew = Eigen::skew(Pcx);

    Eigen::Matrix36d J;
    //J.block<3,3>(0,0) = -Rcw;


  }

protected:
  db::MapPoint::Ptr mMapPoint;
  Eigen::Vector6d*  mLwc;
  Eigen::Matrix23d  mJ;
  Eigen::Vector2d   mR;
  Eigen::Vector2d   mZ;

  MEstimator::Ptr mME;
};
}  //namespace toy